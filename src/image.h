#ifndef MUBL_IMAGE_H
#define MUBL_IMAGE_H

#include "mubl.h"

/* Check the application image.  `first` supplies the first MUBL_HOLD_SIZE bytes when the update
 * engine validates an image it has not yet made bootable; NULL reads them from flash. */
void image_check(mubl_app_t* app, const uint8_t* first);

#endif
