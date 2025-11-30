#include <ftxui/dom/elements.hpp>
#include <ftxui/screen/screen.hpp>
#include <ftxui/screen/string.hpp>
#include <ftxui/component/captured_mouse.hpp>
#include <ftxui/component/component.hpp> 
#include <ftxui/component/component_options.hpp> 
#include <ftxui/component/screen_interactive.hpp>
#include <menu.hpp>

namespace menu {

    void display(const std::vector<std::string> &options) {
        auto screen = ftxui::ScreenInteractive::Fullscreen();
        std::vector<ftxui::Component> entries(options.size());
        for (const std::string &str : options) {
            entries.emplace_back(ftxui::MenuEntry(str));
        }
        auto menu = ftxui::Container::Vertical(
            entries
        );
        screen.Loop(menu);
    }

    int select(const std::vector<std::string> &selections) {
        auto screen = ftxui::ScreenInteractive::Fullscreen();
        int selected = 0;
        ftxui::MenuOption option;
        option.on_enter = screen.ExitLoopClosure();
        auto menu = ftxui::Menu(&selections, &selected, option);
        screen.Loop(menu);
        return selected;
    }

} // namespace menu;
