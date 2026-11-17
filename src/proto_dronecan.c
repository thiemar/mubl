/*
 * DroneCAN firmware-update node on sfwa-dronecan's bootloader transport (dc_tiny).
 *
 * Start-up:
 *   - bitrate: from the application's handoff, else MUBL_CAN_BITRATE, else detected by listening
 *     (no ACKs, no transmission) at 1M, 500k, 250k and 125k in turn until a frame is received; a
 *     protocol error or MUBL_CAN_AUTOBAUD_DWELL_MS of silence moves on to the next rate;
 *   - node ID: from the handoff, else MUBL_CAN_NODE_ID, else dynamic allocation following the
 *     allocatee rules A-E of uavcan.protocol.dynamic_node_id.Allocation.
 *
 * Then: NodeStatus once a second (MAINTENANCE, SOFTWARE_UPDATE while updating; vendor status =
 * application check result | stay reason << 8), GetNodeInfo with the application's version and
 * CRC from its descriptor, RestartNode, and BeginFirmwareUpdate.  An update reads the file with one
 * uavcan.protocol.file.Read outstanding at a time, 256 bytes each, with MUBL_DC_READ_RETRIES
 * retries per read; a shorter read ends the file.  The node boots the new image when it validates
 * and stays (health ERROR) when it does not.
 */
#include "hal/hal.h"
#include "links.h"
#include "update.h"

#if MUBL_DRONECAN

#define LINK MUBL_LINK_DRONECAN
#define TAO (!DC_FRAME_FD)
#define PRIORITY 16U          /* medium */
#define PRIORITY_LOW 24U
#define READ_SIZE 256U

#ifndef MUBL_DC_RESTART
#define MUBL_DC_RESTART 1
#endif

enum { OFF = 0, AUTOBAUD = 1, ALLOCATE = 2, READY = 3 };

static const uint32_t rates[] = {1000000, 500000, 250000, 125000};

static struct {
    uint8_t state;
    uint8_t node_id;
    uint8_t preferred;
    uint8_t detected;
    uint8_t rate;             /* autobaud: index into rates */
    uint8_t matched;          /* allocation: unique ID bytes confirmed by the allocator */
    uint8_t followup;         /* allocation: a second-stage request is due at followup_at */
    uint8_t tid_status, tid_alloc, tid_read, tid_log;
    uint8_t updating;
    uint8_t server;
    uint8_t retries;
    uint8_t failed;
    uint8_t path_len;
    uint32_t bitrate;
    uint32_t switch_at, status_at, alloc_at, followup_at, read_at;
    uint32_t offset;          /* next file offset to read */
    uint8_t uid[16];
    uint8_t path[200];
    dc_tiny_t tiny;
    uint8_t rx[DC_TINY_BUFFER_SIZE];
} d;

/* Encoding space, used by one transfer at a time. */
static union {
    struct {
        struct uavcan_protocol_GetNodeInfoResponse msg;
        uint8_t buf[UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_MAX_SIZE];
    } info;
    struct {
        struct uavcan_protocol_file_ReadRequest msg;
        uint8_t buf[UAVCAN_PROTOCOL_FILE_READ_REQUEST_MAX_SIZE];
    } read;
    struct {
        struct uavcan_protocol_debug_LogMessage msg;
        uint8_t buf[UAVCAN_PROTOCOL_DEBUG_LOGMESSAGE_MAX_SIZE];
    } log;
} scratch;

void dc_frame_send(const dc_frame_t* frame)
{
    (void)hal_can_send(frame);
}

static uint32_t random_between(uint32_t lo, uint32_t hi)
{
    return lo + mubl_random() % (hi - lo + 1U);
}

static void status_fields(struct uavcan_protocol_NodeStatus* ns, uint32_t now)
{
    ns->uptime_sec = now / 1000U;
    ns->health = d.failed ? UAVCAN_PROTOCOL_NODESTATUS_HEALTH_ERROR
                 : mubl_app()->status != MUBL_APP_OK ? UAVCAN_PROTOCOL_NODESTATUS_HEALTH_WARNING
                 : UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
    ns->mode = d.updating ? UAVCAN_PROTOCOL_NODESTATUS_MODE_SOFTWARE_UPDATE : UAVCAN_PROTOCOL_NODESTATUS_MODE_MAINTENANCE;
    ns->sub_mode = 0;
    ns->vendor_specific_status_code = (uint16_t)(mubl_app()->status | (mubl_stay_reason() << 8));
}

static void send_status(uint32_t now)
{
    struct uavcan_protocol_NodeStatus ns;
    uint8_t buf[UAVCAN_PROTOCOL_NODESTATUS_MAX_SIZE];
    status_fields(&ns, now);
    const int32_t n = dc_encode_uavcan_protocol_NodeStatus(&ns, buf, TAO);
    if (n < 0) return;
    dc_tiny_send(&d.tiny, DC_TRANSFER_MESSAGE, UAVCAN_PROTOCOL_NODESTATUS_ID, UAVCAN_PROTOCOL_NODESTATUS_CRC_SEED,
                 0, PRIORITY, d.tid_status++, buf, (uint16_t)n);
}

static void log_text(uint8_t level, const char* text)
{
#if MUBL_DC_LOG
    struct uavcan_protocol_debug_LogMessage* m = &scratch.log.msg;
    memset(m, 0, sizeof *m);
    m->level.value = level;
    memcpy(m->source.data, "mubl", 4);
    m->source.len = 4;
    while (text[m->text.len] != '\0' && m->text.len < sizeof m->text.data) {
        m->text.data[m->text.len] = (uint8_t)text[m->text.len];
        m->text.len++;
    }
    const int32_t n = dc_encode_uavcan_protocol_debug_LogMessage(m, scratch.log.buf, TAO);
    if (n < 0) return;
    dc_tiny_send(&d.tiny, DC_TRANSFER_MESSAGE, UAVCAN_PROTOCOL_DEBUG_LOGMESSAGE_ID,
                 UAVCAN_PROTOCOL_DEBUG_LOGMESSAGE_CRC_SEED, 0, PRIORITY_LOW, d.tid_log++, scratch.log.buf, (uint16_t)n);
#else
    (void)level;
    (void)text;
#endif
}

static void respond(const dc_transfer_t* t, uint16_t id, uint16_t seed, const uint8_t* buf, int32_t n)
{
    if (n < 0) return;
    dc_tiny_send(&d.tiny, DC_TRANSFER_RESPONSE, id, seed, t->source, t->priority, t->transfer_id, buf, (uint16_t)n);
}

static void on_get_node_info(const dc_transfer_t* t, uint32_t now)
{
    struct uavcan_protocol_GetNodeInfoResponse* r = &scratch.info.msg;
    const mubl_app_t* app = mubl_app();
    memset(r, 0, sizeof *r);
    status_fields(&r->status, now);
    if (app->status == MUBL_APP_OK) {
        r->software_version.major = app->major;
        r->software_version.minor = app->minor;
        if (app->format == MUBL_FMT_AP || app->format == MUBL_FMT_LEGACY) {
            r->software_version.optional_field_flags = UAVCAN_PROTOCOL_SOFTWAREVERSION_OPTIONAL_FIELD_FLAG_VCS_COMMIT |
                                                       UAVCAN_PROTOCOL_SOFTWAREVERSION_OPTIONAL_FIELD_FLAG_IMAGE_CRC;
            r->software_version.vcs_commit = app->vcs_commit;
            r->software_version.image_crc = app->crc;
        }
    }
    r->hardware_version.major = MUBL_HW_VERSION_MAJOR;
    r->hardware_version.minor = MUBL_HW_VERSION_MINOR;
    memcpy(r->hardware_version.unique_id, d.uid, sizeof d.uid);
    static const char name[] = MUBL_BOARD_NAME;
    MUBL_STATIC_ASSERT(sizeof name - 1U <= sizeof r->name.data, "MUBL_BOARD_NAME is too long");
    memcpy(r->name.data, name, sizeof name - 1U);
    r->name.len = sizeof name - 1U;
    respond(t, UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_ID, UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_CRC_SEED,
            scratch.info.buf, dc_encode_uavcan_protocol_GetNodeInfoResponse(r, scratch.info.buf, TAO));
#if MUBL_DC_HOLD_ON_GETNODEINFO
    mubl_hold(LINK);
#endif
}

static void send_read(uint32_t now)
{
    struct uavcan_protocol_file_ReadRequest* q = &scratch.read.msg;
    q->offset = d.offset;
    q->path.path.len = d.path_len;
    memcpy(q->path.path.data, d.path, d.path_len);
    const int32_t n = dc_encode_uavcan_protocol_file_ReadRequest(q, scratch.read.buf, TAO);
    d.tid_read = (uint8_t)((d.tid_read + 1U) & 0x1FU);
    d.read_at = now + MUBL_DC_READ_TIMEOUT_MS;
    if (n < 0) return;
    dc_tiny_send(&d.tiny, DC_TRANSFER_REQUEST, UAVCAN_PROTOCOL_FILE_READ_REQUEST_ID,
                 UAVCAN_PROTOCOL_FILE_READ_REQUEST_CRC_SEED, d.server, PRIORITY, d.tid_read, scratch.read.buf, (uint16_t)n);
}

static void fail(const char* why)
{
    update_abort(LINK);
    d.updating = 0;
    d.failed = 1;
    log_text(UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_ERROR, why);
}

static void begin_update(uint32_t now)
{
    d.failed = 0;
    mubl_hold(LINK);
    if (update_begin(LINK) != MUBL_OK) {
        fail("erase failed");
        return;
    }
    d.updating = 1;
    d.offset = 0;
    d.retries = MUBL_DC_READ_RETRIES;
    log_text(UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_INFO, "update started");
    send_status(now);
    send_read(mubl_now());
}

static void on_begin_firmware_update(const dc_transfer_t* t, uint32_t now)
{
    struct uavcan_protocol_file_BeginFirmwareUpdateRequest_view v;
    uint8_t buf[2];
    if (dc_view_uavcan_protocol_file_BeginFirmwareUpdateRequest(t->payload, t->length, &v, TAO) < 0) return;
    uint8_t error = UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_RESPONSE_ERROR_OK;
    const uint16_t len = v.image_file_remote_path.path.len;
    if (d.updating) {
        error = UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_RESPONSE_ERROR_IN_PROGRESS;
    } else if (update_active() || len == 0U || len > sizeof d.path) {
        error = UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_RESPONSE_ERROR_INVALID_MODE;   /* another link is updating */
    }
    /* error, then an empty optional_error_message (TAO: no length byte on classic CAN) */
    buf[0] = error;
    buf[1] = 0;
    respond(t, UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_RESPONSE_ID,
            UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_RESPONSE_CRC_SEED, buf, TAO ? 1 : 2);
    if (error != UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_RESPONSE_ERROR_OK) return;
    d.server = v.source_node_id != 0U ? v.source_node_id : t->source;
    d.path_len = (uint8_t)len;
    memcpy(d.path, v.image_file_remote_path.path.data, len);
    begin_update(now);
}

static void on_read(const dc_transfer_t* t, uint32_t now)
{
    struct uavcan_protocol_file_ReadResponse_view v;
    if (!d.updating || t->source != d.server || t->transfer_id != d.tid_read) return;
    if (dc_view_uavcan_protocol_file_ReadResponse(t->payload, t->length, &v, TAO) < 0 || v.error.value != 0) {
        d.read_at = now;                      /* treated like a lost response: retry */
        return;
    }
    const int r = update_write(LINK, d.offset, v.data.data, v.data.len);
    if (r != MUBL_OK) {
        fail(r == MUBL_E_RANGE ? "image too large" : "flash write failed");
        return;
    }
    d.offset += v.data.len;
    d.retries = MUBL_DC_READ_RETRIES;
    if (v.data.len == READ_SIZE) {
        send_read(mubl_now());
        return;
    }
    d.updating = 0;
    if (update_finish(LINK) != MUBL_OK) {
        d.failed = 1;
        log_text(UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_ERROR, "image rejected");
        return;
    }
    log_text(UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_INFO, "update complete");
    mubl_boot(100);
}

#if MUBL_DC_RESTART
static void on_restart(const dc_transfer_t* t)
{
    struct uavcan_protocol_RestartNodeRequest q;
    struct uavcan_protocol_RestartNodeResponse r;
    uint8_t buf[UAVCAN_PROTOCOL_RESTARTNODE_RESPONSE_MAX_SIZE];
    r.ok = dc_decode_uavcan_protocol_RestartNodeRequest(t->payload, t->length, &q, TAO) >= 0 &&
           q.magic_number == UAVCAN_PROTOCOL_RESTARTNODE_REQUEST_MAGIC_NUMBER;
    respond(t, UAVCAN_PROTOCOL_RESTARTNODE_RESPONSE_ID, UAVCAN_PROTOCOL_RESTARTNODE_RESPONSE_CRC_SEED, buf,
            dc_encode_uavcan_protocol_RestartNodeResponse(&r, buf, TAO));
    if (!r.ok) return;
    if (d.updating) fail("restarted");
    mubl_restart(50);
}
#endif

static void set_node_id(uint8_t id, uint32_t now)
{
    d.node_id = id;
    d.state = READY;
    d.status_at = now;
    dc_tiny_init(&d.tiny, id, d.rx, sizeof d.rx);
    hal_can_filter(id);
    mubl_session()->can_node_id = id;
    const mubl_request_t* req = mubl_request();
    if (req->link == LINK && req->can_server_node_id != 0U && req->path_len != 0U) {
        d.server = req->can_server_node_id;
        d.path_len = req->path_len;
        memcpy(d.path, req->path, req->path_len);
        begin_update(now);
    }
}

static void send_allocation(void)
{
    struct uavcan_protocol_dynamic_node_id_Allocation a;
    uint8_t buf[UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_SIZE];
    uint8_t n = (uint8_t)(16U - d.matched);
    if (n > UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_LENGTH_OF_UNIQUE_ID_IN_REQUEST) {
        n = UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_LENGTH_OF_UNIQUE_ID_IN_REQUEST;
    }
    a.node_id = d.preferred;
    a.first_part_of_unique_id = d.matched == 0U;
    a.unique_id.len = n;
    memcpy(a.unique_id.data, &d.uid[d.matched], n);
    const int32_t len = dc_encode_uavcan_protocol_dynamic_node_id_Allocation(&a, buf, TAO);
    if (len < 0) return;
    dc_tiny_send(&d.tiny, DC_TRANSFER_MESSAGE, UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID,
                 UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_CRC_SEED, 0, PRIORITY_LOW, d.tid_alloc++, buf, (uint16_t)len);
}

static void on_allocation(const dc_transfer_t* t, uint32_t now)
{
    struct uavcan_protocol_dynamic_node_id_Allocation_view v;
    /* Rule C: any Allocation message restarts the request timer and cancels a pending follow-up. */
    d.alloc_at = now + random_between(UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS,
                                      UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_REQUEST_PERIOD_MS);
    d.followup = 0;
    if (t->source == 0U) return;                         /* another allocatee's request */
    if (dc_view_uavcan_protocol_dynamic_node_id_Allocation(t->payload, t->length, &v, TAO) < 0) return;
    const uint16_t n = v.unique_id.len;
    if (n == 0U || n > 16U || memcmp(v.unique_id.data, d.uid, n) != 0) {
        d.matched = 0;
        return;
    }
    if (n < 16U) {                                       /* Rule D */
        d.matched = (uint8_t)n;
        d.followup = 1;
        d.followup_at = now + random_between(UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_FOLLOWUP_DELAY_MS,
                                             UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_FOLLOWUP_DELAY_MS);
    } else if (v.node_id != 0U) {                        /* Rule E */
        set_node_id(v.node_id, now);
    }
}

static void dispatch(const dc_transfer_t* t, uint32_t now)
{
    const uint16_t id = t->data_type_id;
    if (t->transfer == DC_TRANSFER_MESSAGE) {
        if (id == UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID && d.state == ALLOCATE) on_allocation(t, now);
    } else if (t->transfer == DC_TRANSFER_RESPONSE) {
        if (id == UAVCAN_PROTOCOL_FILE_READ_RESPONSE_ID) on_read(t, now);
    } else if (id == UAVCAN_PROTOCOL_GETNODEINFO_REQUEST_ID) {
        on_get_node_info(t, now);
    } else if (id == UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_REQUEST_ID) {
        on_begin_firmware_update(t, now);
#if MUBL_DC_RESTART
    } else if (id == UAVCAN_PROTOCOL_RESTARTNODE_REQUEST_ID) {
        on_restart(t);
#endif
    }
}

/* The bitrate is known: run normally, then get a node ID. */
static void online(uint32_t bitrate, uint32_t now)
{
    d.bitrate = bitrate;
    mubl_session()->can_bitrate = bitrate;
    hal_can_start(bitrate, false);
    dc_tiny_init(&d.tiny, 0, d.rx, sizeof d.rx);
    if (d.node_id != 0U) {
        set_node_id(d.node_id, now);
    } else {
        d.state = ALLOCATE;
        d.matched = 0;
        d.alloc_at = now + random_between(UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS,
                                          UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_REQUEST_PERIOD_MS);
    }
}

static void listen_at(uint8_t index, uint32_t now)
{
    d.rate = index;
    d.switch_at = now + MUBL_CAN_AUTOBAUD_DWELL_MS;
    hal_can_start(rates[index], true);
    (void)hal_can_error();
}

void dronecan_poll(uint32_t now)
{
    dc_frame_t f;
    dc_transfer_t t;
    if (d.state == OFF) return;
    if (d.state == AUTOBAUD) {
        if (hal_can_receive(&f)) {
            d.detected = 1;
            mubl_detected(LINK);
            online(rates[d.rate], now);
        } else if (hal_can_error() || (int32_t)(now - d.switch_at) >= 0) {
            listen_at((uint8_t)((d.rate + 1U) % (sizeof rates / sizeof rates[0])), now);
        }
        return;
    }
    while (hal_can_receive(&f)) {
        if (!d.detected) {
            d.detected = 1;
            mubl_detected(LINK);
        }
        if (dc_tiny_receive(&d.tiny, &f, now, &t)) dispatch(&t, now);
    }
    now = mubl_now();
    if (d.state == ALLOCATE) {
        if (d.followup && (int32_t)(now - d.followup_at) >= 0) {
            d.followup = 0;
            send_allocation();                            /* Rule D, second stage */
        }
        if ((int32_t)(now - d.alloc_at) >= 0) {           /* Rule B */
            d.alloc_at = now + random_between(UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS,
                                              UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_REQUEST_PERIOD_MS);
            d.matched = 0;
            send_allocation();
        }
        return;
    }
    if ((int32_t)(now - d.status_at) >= 0) {
        d.status_at = now + 1000U;
        send_status(now);
    }
    if (d.updating && (int32_t)(now - d.read_at) >= 0) {
        if (d.retries == 0U) {
            fail("file server not responding");
        } else {
            d.retries--;
            send_read(now);
        }
    }
}

void dronecan_start(void)
{
    const mubl_request_t* req = mubl_request();
    const uint32_t now = mubl_now();
    memset(&d, 0, sizeof d);
    hal_unique_id(d.uid);
    d.node_id = req->can_node_id != 0U ? req->can_node_id : MUBL_CAN_NODE_ID;
    d.preferred = MUBL_CAN_PREFERRED_NODE_ID;
    if (req->can_bitrate != 0U) {
        online(req->can_bitrate, now);
    } else if (MUBL_CAN_BITRATE != 0) {
        online(MUBL_CAN_BITRATE, now);
    } else {
        d.state = AUTOBAUD;
        listen_at(0, now);
    }
}

void dronecan_stop(void)
{
    if (d.updating) update_abort(LINK);
    d.updating = 0;
    d.state = OFF;
    hal_can_stop();
}

#endif
