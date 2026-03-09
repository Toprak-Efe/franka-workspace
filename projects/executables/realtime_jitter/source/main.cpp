#include "matplot/freestanding/plot.h"
#include <chrono>
#include <ctime>
#include <datalog.hpp>
#include <matplot/matplot.h>
#include <nanoloop.hpp>
#include <vector>

volatile bool g_running{true};

void signal_handler(int signal_no) { g_running = false; }

struct ThreadLoopMeasure {
  std::chrono::steady_clock::time_point begin, end;
};

int main(int argc, char *argv[]) {
  constexpr std::size_t LOOP_COUNT = 1000;

  std::vector<ThreadLoopMeasure> measures;
  measures.reserve(LOOP_COUNT);

  std::size_t iteration{0};
  Nanoloop<1000000> milisecond_loop;

  while (g_running && (iteration++ != LOOP_COUNT)) {
    ThreadLoopMeasure measure{.begin{std::chrono::steady_clock::now()}, .end{}};

    measure.end = std::chrono::steady_clock::now();
    measures.emplace_back(measure);
    milisecond_loop.await();
  }

  std::vector<double> loop_times;
  loop_times.reserve(measures.size());

  for (const auto &tlm : measures) {
    std::chrono::duration<double, std::micro> ms_double = tlm.end - tlm.begin;
    loop_times.emplace_back(ms_double.count());
  }

  auto h = matplot::hist(loop_times);
  h->num_bins(32);
  h->face_color("#0072BD");
  h->edge_color("white");
  h->line_width(1.5);

  matplot::title("Performance: Loop Execution Times");
  matplot::xlabel("Time (us)");
  matplot::ylabel("Frequency");

  matplot::grid(matplot::on);

  matplot::show();

  return 0;
}
