#include "stf_dependancy.h"

__weak void exception(std::string str) {
    UNUSED(str);
}

__weak void notify(const char* str) {
    UNUSED(str);
}
