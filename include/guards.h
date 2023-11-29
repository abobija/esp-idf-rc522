#pragma once

#include <esp_err.h>

/**
 * @brief Label for success guard gate in macros.
 *        Used as a goto label to handle successful execution paths in macros.
 */
#define SUCCESS_GUARD_GATE __success_guard_gate

/**
 * @brief Label for error guard gate in macros.
 *        Used as a goto label to handle error paths in macros.
 */
#define ERROR_GUARD_GATE __error_guard_gate

/**
 * @brief Label for exit guard gate in macros.
 *        Used as a goto label to mark the end of execution paths in macros.
 */
#define EXIT_GUARD_GATE __exit_guard_gate

/**
 * @brief Macro to create jump points for success and error handling.
 *        This macro defines labels for success and error handling, and a common exit point.
 * @param on_error Code to execute on error.
 * @param on_success Code to execute on success.
 */
#define JMP_GUARD_GATES(on_error, on_success) \
    goto SUCCESS_GUARD_GATE; \
        ERROR_GUARD_GATE: { on_error; goto EXIT_GUARD_GATE; };  \
        SUCCESS_GUARD_GATE: { on_success; goto EXIT_GUARD_GATE; };  \
        EXIT_GUARD_GATE: {};

/**
 * @brief Macro guard for jumping to an error handler on ESP error.
 *        Checks the result of EXP and if it's not ESP_OK, jumps to ERROR_GUARD_GATE.
 * @param EXP Expression that returns an esp_err_t value.
 */
#define ESP_ERR_JMP_GUARD(EXP) \
    if((err = (EXP)) != ESP_OK) { goto ERROR_GUARD_GATE; }

/**
 * @brief Macro for logging an error message and jumping to an error handler on ESP error.
 *        If EXP is not ESP_OK, logs the provided message and jumps to ERROR_GUARD_GATE.
 * @param EXP Expression that returns an esp_err_t value.
 * @param message The message to log on error.
 */
#define ESP_ERR_LOG_AND_JMP_GUARD(EXP, message) \
    if((err = (EXP)) != ESP_OK) { ESP_LOGE(TAG, message); goto ERROR_GUARD_GATE; }

/**
 * @brief Macro for conditionally logging an error message and jumping to an error handler.
 *        If EXP evaluates to true, logs the provided message and jumps to ERROR_GUARD_GATE.
 * @param EXP Boolean expression to evaluate.
 * @param message The message to log if EXP is true.
 */
#define CONDITION_LOG_AND_JMP_GUARD(EXP, message) \
    if(EXP) { ESP_LOGE(TAG, message); goto ERROR_GUARD_GATE; }

/**
 * @brief Macro guard for memory allocation with jump to error handling.
 *        This macro checks if a memory allocation expression (EXP) fails (i.e., returns NULL).
 *        If the allocation fails, it jumps to the ERROR_GUARD_GATE label.
 *        This is typically used in functions where error handling is centralized
 *        at a specific label (ERROR_GUARD_GATE) for cleaner and more manageable code.
 *        It helps in avoiding deep nesting of error checks after each allocation.
 * @param EXP Memory allocation expression (e.g., a call to malloc).
 */
#define ALLOC_JMP_GUARD(EXP) \
    if((EXP) == NULL) { err = ESP_ERR_NO_MEM; goto ERROR_GUARD_GATE; }

/**
 * @brief Macro guard for memory allocation functions.
 *        This macro ensures that if a memory allocation expression fails (returns NULL),
 *        the function in which this macro is used will immediately return with ESP_ERR_NO_MEM.
 *        It's useful for handling memory allocation failures gracefully in functions.
 * @param EXP Memory allocation expression (e.g., a call to malloc).
 * @return Exits the caller function with ESP_ERR_NO_MEM if EXP evaluates to NULL.
 */
#define ALLOC_RET_GUARD(EXP) \
    if((EXP) == NULL) { return ESP_ERR_NO_MEM; }

/**
 * @brief Macro guard for ESP error handling.
 *        This macro checks if the result of EXP is not ESP_OK and, if so, returns the error code.
 * @param EXP Expression that returns an esp_err_t value.
 * @return Returns the error code if EXP is not ESP_OK.
 */
#define ESP_ERR_RET_GUARD(EXP) \
    if((err = (EXP)) != ESP_OK) { return err; }
