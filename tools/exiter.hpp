#ifndef TOOLS__EXITER_HPP
#define TOOLS__EXITER_HPP

namespace tools
{
bool exiting();

class Exiter
{
public:
  Exiter();

  bool exit() const;
};

}  // namespace tools

#endif  // TOOLS__EXITER_HPP