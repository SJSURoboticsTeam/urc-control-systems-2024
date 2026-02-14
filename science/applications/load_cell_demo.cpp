#include <array>
#include <cinttypes>

#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>

#include <resource_list.hpp>
#include <tuple>

const int sample_rate = 20;
const float filter_const[20] = { 9.66174445e-05, 1.55986763e-04, 3.33618583e-04,
                                 7.15587299e-04, 1.46574468e-03, 2.93946352e-03,
                                 6.05407192e-03, 1.38743094e-02, 4.26119415e-02,
                                 4.02822938e-01, 4.02822938e-01, 4.26119415e-02,
                                 1.38743094e-02, 6.05407192e-03, 2.93946352e-03,
                                 1.46574468e-03, 7.15587299e-04, 3.33618583e-04,
                                 1.55986763e-04, 9.66174445e-05 };

using namespace std::chrono_literals;
namespace sjsu::science {

float filter(std::array<float, sample_rate> arr, int begin)
{
  float sum = 0;
  for (uint16_t i = 0; i < sample_rate; i++) {
    int idx = (begin + i) % sample_rate;
    sum += (arr[idx] * filter_const[i]);
  }
  return sum;
}

void application()
{
  auto clock = resources::clock();
  auto console = resources::console();
  auto adc = resources::adc_0();

  hal::print(*console, "ADC Application Starting...\n");

  // float adc_data[sample_rate] = { 0 };
  std::array<float, sample_rate> adc_data;
  adc_data.fill(0);

  int begin = 0;

  while (true) {
    adc_data[begin] = adc->read();
    hal::print<128>(
      *console, "LOAD CELL PERCENT: %f\n", filter(adc_data, begin));
    hal::delay(*clock, 100ms);
    begin++;

    if (begin > sample_rate-1) {
      begin = 0;
    }
  }
}

}  // namespace sjsu::science