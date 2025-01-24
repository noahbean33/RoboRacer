/*
1. Define constants:
   - FIXED_POINT_FRACTIONAL_BITS = 16

2. Define LowPassFilter structure:
   - alpha: Smoothing factor (fixed-point)
   - y_prev: Previous output sample (fixed-point)

3. Function lp_filter_init(filter, cutoff_freq, sample_rate):
   - If cutoff_freq <= 0 or sample_rate <= 0:
     - Return error (invalid parameters)
   - Compute dt = 1.0 / sample_rate
   - Compute RC = 1.0 / (2 * π * cutoff_freq)
   - Compute alpha = dt / (RC + dt)
   - Convert alpha to fixed-point and store in filter.alpha
   - Initialize filter.y_prev = 0
   - Return success

4. Function lp_filter_apply(filter, x):
   - Compute y = y_prev + ((alpha * (x - y_prev)) >> FIXED_POINT_FRACTIONAL_BITS)
   - Update y_prev = y
   - Return y

5. Main function:
   - Define cutoff_frequency and sampling_rate
   - Create LowPassFilter instance
   - Initialize filter using lp_filter_init
   - If initialization fails:
     - Print error message and exit
   - Define input_signal array (simulated or real sensor data)
   - For each sample in input_signal:
     - Apply filter using lp_filter_apply
     - Store output in output_signal
     - Print input and output
*/

#include <stdio.h>
#include <stdint.h>
#include <math.h>

/* Fixed-point precision */
#define FIXED_POINT_FRACTIONAL_BITS 16
#define FLOAT_TO_FIXED(x) ((int32_t)((x) * (1 << FIXED_POINT_FRACTIONAL_BITS)))
#define FIXED_TO_FLOAT(x) ((double)(x) / (1 << FIXED_POINT_FRACTIONAL_BITS))

/* Low-Pass Filter Structure */
typedef struct {
    int32_t alpha;      /* Smoothing factor (fixed-point) */
    int32_t y_prev;     /* Previous output sample (fixed-point) */
} LowPassFilter;

/**
 * Initialize the low-pass filter.
 *
 * @param filter        Pointer to the LowPassFilter structure
 * @param cutoff_freq   Cutoff frequency of the filter in Hz
 * @param sample_rate   Sampling rate in Hz
 * @return              0 on success, -1 on error
 */
int lp_filter_init(LowPassFilter *filter, double cutoff_freq, double sample_rate) {
    if (cutoff_freq <= 0 || sample_rate <= 0) {
        return -1;  /* Invalid parameters */
    }

    /* Calculate the time constant (RC) */
    double dt = 1.0 / sample_rate;
    double RC = 1.0 / (2.0 * M_PI * cutoff_freq);

    /* Compute the smoothing factor alpha */
    filter->alpha = FLOAT_TO_FIXED(dt / (RC + dt));

    /* Initialize previous output to zero */
    filter->y_prev = 0;

    return 0;
}

/**
 * Apply the low-pass filter to a single input sample.
 *
 * @param filter    Pointer to the LowPassFilter structure
 * @param x         Current input sample (fixed-point)
 * @return          Filtered output sample (fixed-point)
 */
int32_t lp_filter_apply(LowPassFilter *filter, int32_t x) {
    /* Low-pass filter difference equation */
    int32_t y = filter->y_prev + ((filter->alpha * (x - filter->y_prev)) >> FIXED_POINT_FRACTIONAL_BITS);

    /* Update the previous output */
    filter->y_prev = y;

    return y;
}

/* Example usage */
int main() {
    LowPassFilter filter;
    double cutoff_frequency = 1000.0; /* Cutoff frequency in Hz (e.g., 1 kHz) */
    double sampling_rate = 100000.0;  /* Sampling rate in Hz (e.g., 100 kHz) */

    /* Initialize the filter */
    if (lp_filter_init(&filter, cutoff_frequency, sampling_rate) != 0) {
        fprintf(stderr, "Error: Invalid filter parameters.\n");
        return 1;
    }

    /* Simulated input data (replace with actual sensor data) */
    int32_t input_signal[] = {
        FLOAT_TO_FIXED(0.0), FLOAT_TO_FIXED(0.5), FLOAT_TO_FIXED(1.0),
        FLOAT_TO_FIXED(0.5), FLOAT_TO_FIXED(0.0), FLOAT_TO_FIXED(-0.5),
        FLOAT_TO_FIXED(-1.0), FLOAT_TO_FIXED(-0.5), FLOAT_TO_FIXED(0.0)
    };
    int num_samples = sizeof(input_signal) / sizeof(int32_t);
    int32_t output_signal[num_samples];

    /* Apply the filter to each sample */
    for (int i = 0; i < num_samples; i++) {
        output_signal[i] = lp_filter_apply(&filter, input_signal[i]);
        printf("Input: %d, Output: %d\n", input_signal[i], output_signal[i]);
    }

    return 0;
}
