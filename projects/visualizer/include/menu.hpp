#ifndef MENU_HPP
#define MENU_HPP

#include <vector>
#include <string>
#include <ftxui/dom/elements.hpp>
#include <ftxui/screen/screen.hpp>
#include <ftxui/screen/string.hpp>
#include <ftxui/component/captured_mouse.hpp>
#include <ftxui/component/component.hpp> 
#include <ftxui/component/component_options.hpp> 
#include <ftxui/component/screen_interactive.hpp>

namespace menu {

void display(const std::vector<std::string> &options);

int select(const std::vector<std::string> &options);

} // namespace menu;

#endif//MENU_HPP
