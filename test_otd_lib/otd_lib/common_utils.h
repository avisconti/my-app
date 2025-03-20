/**
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#ifndef COMMON_UTILS_H
#define COMMON_UTILS_H

#include <stdint.h>

/**
 * @brief Get the version of the library.
 *
 * @param version Pointer to version string of at least 12 char
 * @param length Length of the provided version array
 * @return Flag indicating if the provided length is sufficient.
 */
uint8_t common_utils_get_version(char *version, uint8_t length);

#endif /* COMMON_UTILS_H */
