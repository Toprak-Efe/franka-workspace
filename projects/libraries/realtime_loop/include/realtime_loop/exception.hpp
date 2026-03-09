#include <stdexcept>

namespace Asclepius {

class AsclepiusException : public std::exception {
public:
  AsclepiusException(const std::string &);
  const char *what() const noexcept; 
private:
    std::string m_msg;
};

}; // namespace Asclepius
