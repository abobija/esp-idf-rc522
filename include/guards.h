#pragma once

#include <esp_err.h>

/**
 * @brief Macro guard for *alloc functions.
 *        Wrapping memory allocation expression inside of this macro will ensure that
 *        function, in which this macro is called, is going to immediately return
 *        ESP_ERR_NO_MEM if *alloc function fail to reserve the memory.
 * @param EXP Memory allocation expression
 * @return Exit from caller if EXP returns NULL.
*/
#define ALLOC_RET_GUARD(EXP) \
    if((EXP) == NULL) { return ESP_ERR_NO_MEM; }

#define ESP_ERR_RET_GUARD(EXP) \
    if((err = (EXP)) != ESP_OK) { return err; }

#define SUCCESS_GUARD_GATE __success_guard_gate
#define ERROR_GUARD_GATE __error_guard_gate
#define EXIT_GUARD_GATE __exit_guard_gate

#define JMP_GUARD_GATES(on_error, on_success) \
    goto SUCCESS_GUARD_GATE; \
        ERROR_GUARD_GATE: { on_error; goto EXIT_GUARD_GATE; };  \
        SUCCESS_GUARD_GATE: { on_success; goto EXIT_GUARD_GATE; };  \
        EXIT_GUARD_GATE: {};

#define ESP_ERR_JMP_GUARD(EXP) \
    if((err = (EXP)) != ESP_OK) { goto ERROR_GUARD_GATE; }

#define ESP_ERR_LOG_AND_JMP_GUARD(EXP, message) \
    if((err = (EXP)) != ESP_OK) { ESP_LOGE(TAG, message); goto ERROR_GUARD_GATE; }

#define CONDITION_LOG_AND_JMP_GUARD(EXP, message) \
    if(EXP) { ESP_LOGE(TAG, message); goto ERROR_GUARD_GATE; }
