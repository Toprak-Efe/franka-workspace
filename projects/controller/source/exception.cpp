#include <exception.hpp>


Asclepius::AsclepiusException::AsclepiusException(const std::string &msg) : m_msg(msg) {} 

const char *Asclepius::AsclepiusException::what() const noexcept {
    return m_msg.c_str();
}
