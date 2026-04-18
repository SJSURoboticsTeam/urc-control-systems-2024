#include <array>
#include <cinttypes>

#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>

#include <resource_list.hpp>

constexpr int sample_rate = 40;
/*Constants for digital filter obtained though signal.firwin2(20, [0, fs / 2], [1.0, 0], fs = fs) 
20 is number of sample and fs is sample frequency: 1/100ms*/
constexpr std::array<float, sample_rate> filter_const = {
  2.30217199e-05, 2.72703862e-05, 3.64891392e-05, 5.21727953e-05,
  7.62544386e-05, 1.11288539e-04, 1.60733417e-04, 2.29399083e-04,
  3.24180225e-04, 4.55301222e-04, 6.38525399e-04, 8.99285005e-04,
  1.28090508e-03, 1.86230934e-03, 2.80011714e-03, 4.44271871e-03,
  7.70103376e-03, 1.56342679e-02, 4.44497885e-02, 4.04700512e-01,
  4.04700512e-01, 4.44497885e-02, 1.56342679e-02, 7.70103376e-03,
  4.44271871e-03, 2.80011714e-03, 1.86230934e-03, 1.28090508e-03,
  8.99285005e-04, 6.38525399e-04, 4.55301222e-04, 3.24180225e-04,
  2.29399083e-04, 1.60733417e-04, 1.11288539e-04, 7.62544386e-05,
  5.21727953e-05, 3.64891392e-05, 2.72703862e-05, 2.30217199e-05
};

using namespace std::chrono_literals;
namespace sjsu::science {

// Digital fileter
// arr: buffer, begin: pointer to the start of the  
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

  std::array<float, sample_rate> adc_data;
  adc_data.fill(0);
  int begin = 0;

  // read value from adc
  //  values buffered into array and digital
  while (true) {
    adc_data[begin] = adc->read();
    hal::print<128>(
      *console, "LOAD CELL PERCENT: %f\n", filter(adc_data, begin));
    hal::delay(*clock, 100ms);
    begin++;

    if (begin > sample_rate - 1) {
      begin = 0;
    }
  }
}

}  // namespace sjsu::science