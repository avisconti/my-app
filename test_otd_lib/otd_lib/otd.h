/**
 * @file otd.h
 * @brief On Table Detection Library.
 * @details Detects on-table condition using IMU (A + G) or accelerometer sensor.
 * @author Stefano Paolo Rivolta
 * @author Piergiorgio Arrigoni
 * @date 2023
 * @copyright STMicroelectronics
 */

#ifndef _OTD_H_
#define _OTD_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief Output values.
 */
typedef enum {
	OTD_INIT_SUCCESS = 0,
	OTD_INIT_ERROR_HYST = 1,
	OTD_INIT_ERROR_META = 2,
	OTD_INIT_ERROR_MODEL = 3
} otd_init_status_t;

/**
 * @brief Output values.
 */
typedef enum {
	OTD_UNKNOWN = 0,
	OTD_ON_TABLE = 1,
	OTD_ON_LAP = 2,   // For OTD 1.0 models, this output refers to the "Not-on-table" state
	OTD_OTHER = 3     // For OTD 1.0 models, this output is not possible
} otd_output_t;

/**
 * @brief Supported models.
 */
typedef enum {
	OTD_MODEL_0 = 0, // OTD 2.0 model suitable for LSM6DSL, LSM6DSOETR3, LSM6DSOX or LSM6DSO16IS mounted on the screen panel
	OTD_MODEL_1 = 1, // OTD 1.0 model suitable for LIS2DW12 mounted on the keyboard panel
	OTD_MODEL_2 = 2  // OTD 2.0 model suitable for LIS2DUX(S)12 mounted on the keyboard panel
} otd_model_t;

/**
 * @brief Metaclassification mechanisms.
 */
typedef enum {
	OTD_META_RESET = 0,
	OTD_META_DECREMENT = 1
} otd_meta_t;

/**
 * @brief Configuration structure.
 */
typedef struct {
	float hyst_ontable_ths;    /**< [s] On-table hysteresis threshold. */
	float hyst_onlap_ths;      /**< [s] On-lap hysteresis threshold. */
	float hyst_other_ths;      /**< [s] Other hysteresis threshold. */
} otd_config_t;

/**
 * @brief Counters structure.
 * @details Contains the values of the counters reached by the meta-classifier.
 */
typedef struct {
	float hyst_ontable_cnt;    /**< [s] On-table hysteresis counter. */
	float hyst_onlap_cnt;      /**< [s] On-lap hysteresis counter. */
	float hyst_other_cnt;      /**< [s] Other hysteresis counter. */
} otd_counters_t;

/**
 * @brief Input data structure.
 * @details Contain the input data that is provided at each step of the algorithm, which must be in ENU orientation
 */
typedef struct {
	float acc[3];  /**< [mg] Accelerometer values. */
	float gyro[3]; /**< [dps] Uncalibrated gyroscope values. If not used, fill with zeros */
} otd_input_t;

/**
 * @brief State data structure.
 */
typedef struct otd_struct_state_t otd_state_t;

/**
 * @brief Get algorithm instance.
 *
 * @param instance_index Index of the algorithm instance
 * @return Pointer to the algorithm instance (NULL if not enough instances).
 */
otd_state_t *otd_get_instance(uint8_t instance_index);

/**
 * @brief Get default configuration.
 *
 * @return Configuration structure containing the default values.
 */
otd_config_t otd_get_default_config(void);

/**
 * @brief Initialize algorithm.
 * @details Initialize or reset the library.
 *
 * @param state Pointer to the algorithm instance
 * @param config Pointer to the configuration structure
 * @param meta Output stabilizer method
 * @param model OTD AI model to be executed
 */
otd_init_status_t otd_init(otd_state_t *state, const otd_config_t *config, otd_meta_t meta, otd_model_t model);

/**
 * @brief Algorithm run.
 * @details Run one step of the algorithm. It must be called at 50 Hz pace.
 *
 * @param state Pointer to the algorithm instance
 * @param output_raw Pointer to the enum where the algorithm will save the raw results
 * @param output_meta Pointer to the enum where the algorithm will save the metaclassified results
 * @param input Pointer to the struct containing the data to be provided to the algorithm 
 * @return Flag indicating if a new window has been classified.
 */
uint8_t otd_run(otd_state_t *state, otd_output_t *output_raw, otd_output_t *output_meta, const otd_input_t *input);

/**
 * @brief Get metaclassifiers stability flag after running one step of the algorithm.
 *
 * @param state Pointer to the algorithm instance
 * @param counters Pointer to the counters structure 
 * @return Flag indicating if the metaclassifiers have reached stability.
 */
uint8_t otd_get_meta_stability(otd_state_t *state, otd_counters_t *counters);

/**
 * @brief Set initial output state.
 *
 * @param state Pointer to the algorithm instance
 * @param init_output Output state to be set
 */
void otd_set_init_output(otd_state_t *state, otd_output_t init_output);

/**
 * @brief Get the hash of the selected model.
 *
 * @param state Pointer to the algorithm instance
 * @return Model hash
 */
uint32_t otd_get_model_hash(otd_state_t *state);

/**
 * @brief Get the version of the library.
 *
 * @param version Pointer to version string of at least 12 char
 * @param length Length of the provided version array
 * @return Flag indicating if the provided length is sufficient.
 */
uint8_t otd_get_version(char *version, uint8_t length);

#ifdef __cplusplus
}
#endif

#endif /* _OTD_H_ */
