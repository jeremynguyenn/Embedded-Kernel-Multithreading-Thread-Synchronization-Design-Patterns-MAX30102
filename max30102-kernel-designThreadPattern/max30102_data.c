#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/math64.h>  // For div64_u64 in calculations
#include "max30102.h"

/* Spinlock for atomic FIFO access to prevent race conditions */
static DEFINE_SPINLOCK(fifo_spinlock);

/**
 * max30102_clear_fifo - Clear FIFO pointers
 * @data: MAX30102 device data
 * Returns: 0 on success, negative error code on failure
 */
static int max30102_clear_fifo(struct max30102_data *data)
{
    uint8_t value = 0x00;
    int ret;

    if (!data) return -EINVAL;

    ret = max30102_write_reg(data, MAX30102_REG_FIFO_WRITE_POINTER, &value, 1);
    if (ret < 0) {
        dev_err(&data->client->dev, "Failed to clear FIFO write pointer: %d\n", ret);
        return ret;
    }
    ret = max30102_write_reg(data, MAX30102_REG_FIFO_READ_POINTER, &value, 1);
    if (ret < 0) {
        dev_err(&data->client->dev, "Failed to clear FIFO read pointer: %d\n", ret);
        return ret;
    }
    ret = max30102_write_reg(data, MAX30102_REG_OVERFLOW_COUNTER, &value, 1);
    if (ret < 0) {
        dev_err(&data->client->dev, "Failed to clear FIFO overflow counter: %d\n", ret);
        return ret;
    }
    return 0;
}

/* Helper function to calculate mean of uint32_t array */
static uint32_t calculate_mean(uint32_t *arr, uint8_t len) {
    uint64_t sum = 0;
    int i;
    for (i = 0; i < len; i++) {
        sum += arr[i];
    }
    return (uint32_t)div64_u64(sum, len);
}

/* Helper function to calculate standard deviation */
static uint32_t calculate_stddev(uint32_t *arr, uint8_t len, uint32_t mean) {
    uint64_t variance = 0;
    int i;
    for (i = 0; i < len; i++) {
        int32_t diff = arr[i] - mean;
        variance += diff * diff;
    }
    variance = div64_u64(variance, len);
    return (uint32_t)int_sqrt(variance);  // Approximate sqrt
}

/* Helper function to find min/max in array */
static uint32_t find_min(uint32_t *arr, uint8_t len) {
    uint32_t min_val = UINT32_MAX;
    int i;
    for (i = 0; i < len; i++) {
        if (arr[i] < min_val) min_val = arr[i];
    }
    return min_val;
}

static uint32_t find_max(uint32_t *arr, uint8_t len) {
    uint32_t max_val = 0;
    int i;
    for (i = 0; i < len; i++) {
        if (arr[i] > max_val) max_val = arr[i];
    }
    return max_val;
}

/**
 * max30102_read_fifo - Read FIFO data (Red and IR samples)
 * @data: MAX30102 device data
 * @red: Buffer for Red LED samples
 * @ir: Buffer for IR LED samples
 * @len: Pointer to store number of samples read
 * Returns: 0 on success, negative error code on failure
 */
int max30102_read_fifo(struct max30102_data *data, uint32_t *red, uint32_t *ir, uint8_t *len)
{
    unsigned long flags;
    uint8_t ovf_counter;
    int ret;

    if (!data || !red || !ir || !len) return -EINVAL;

    if (!data->fifo_full) {
        dev_dbg(&data->client->dev, "No FIFO data available\n");
        return -ENODATA;
    }

    // Check overflow counter as per datasheet
    ret = max30102_read_reg(data, MAX30102_REG_OVERFLOW_COUNTER, &ovf_counter, 1);
    if (ret < 0) {
        dev_err(&data->client->dev, "Failed to read overflow counter: %d\n", ret);
        return ret;
    }
    if (ovf_counter > 0) {
        dev_warn(&data->client->dev, "FIFO overflow: %d samples lost\n", ovf_counter);
        // Auto-adjust sample rate or LED current if overflow (improvement)
        uint8_t config;
        ret = max30102_read_reg(data, MAX30102_REG_SPO2_CONFIG, &config, 1);
        if (ret < 0) return ret;
        config = (config & ~0x1C) | (0x04 << 2);  // Reduce SR to 200sps as example
        ret = max30102_set_spo2_config(data, config);
        if (ret < 0) return ret;
    }

    read_lock(&data->lock);  // Use read_lock for read-only operation
    spin_lock_irqsave(&fifo_spinlock, flags);  // Atomic protection
    memcpy(red, data->red_data, sizeof(data->red_data));
    memcpy(ir, data->ir_data, sizeof(data->ir_data));
    *len = data->data_len;
    data->fifo_full = false;
    spin_unlock_irqrestore(&fifo_spinlock, flags);

    // Auto-clear FIFO after reading
    ret = max30102_clear_fifo(data);
    if (ret < 0) {
        read_unlock(&data->lock);
        return ret;
    }

    read_unlock(&data->lock);

    // Real algorithm for heart rate and SpO2 (replaced placeholder)
    if (data->input_dev && *len > 10) {  // Require at least 10 samples for calculation
        // Calculate heart rate from IR data (simple peak detection)
        uint32_t ir_mean = calculate_mean(ir, *len);
        uint32_t ir_stddev = calculate_stddev(ir, *len, ir_mean);
        uint32_t threshold = ir_mean + ir_stddev / 2;  // Threshold for peak
        int peak_count = 0;
        uint64_t total_interval = 0;
        int last_peak = -1;

        int i;
        for (i = 1; i < *len - 1; i++) {
            if (ir[i] > threshold && ir[i] > ir[i-1] && ir[i] > ir[i+1]) {
                if (last_peak >= 0) {
                    total_interval += (i - last_peak);
                    peak_count++;
                }
                last_peak = i;
            }
        }

        int heart_rate = 0;
        if (peak_count > 0) {
            uint64_t avg_interval = total_interval / peak_count;
            heart_rate = 60 * 100 / avg_interval;  // Sample rate = 100 Hz
        }

        // Calculate SpO2
        uint32_t red_mean = calculate_mean(red, *len);
        uint32_t ir_mean = calculate_mean(ir, *len);
        uint32_t ac_red = find_max(red, *len) - find_min(red, *len);
        uint32_t ac_ir = find_max(ir, *len) - find_min(ir, *len);
        double ratio = (ac_red * 1.0 / red_mean) / (ac_ir * 1.0 / ir_mean);
        int spo2 = (int)(-45.0 * ratio * ratio + 30.0 * ratio + 94.0);  // Approximate calibration formula
        if (spo2 < 0) spo2 = 0;
        if (spo2 > 100) spo2 = 100;

        // Report valid values
        if (heart_rate > 30 && heart_rate < 220 && spo2 > 50 && spo2 <= 100) {
            input_report_abs(data->input_dev, ABS_HEART_RATE, heart_rate);
            input_report_abs(data->input_dev, ABS_SPO2, spo2);
            input_sync(data->input_dev);
            dev_info(&data->client->dev, "Calculated HR: %d bpm, SpO2: %d%%\n", heart_rate, spo2);
        } else {
            dev_warn(&data->client->dev, "Invalid HR/SpO2 calculation, skipping report\n");
        }
    }

    return 0;
}

/**
 * max30102_read_temperature - Read die temperature
 * @data: MAX30102 device data
 * @temp: Pointer to store temperature value
 * Returns: 0 on success, negative error code on failure
 */
int max30102_read_temperature(struct max30102_data *data, float *temp)
{
    uint8_t temp_int, temp_frac, status;
    int ret, timeout = 10;  // Poll instead of fixed sleep, as per datasheet (~29ms)

    if (!data || !temp) return -EINVAL;

    ret = max30102_write_reg(data, MAX30102_REG_DIE_TEMP_CONFIG, & (uint8_t){MAX30102_TEMP_START}, 1);
    if (ret < 0) {
        dev_err(&data->client->dev, "Failed to start temperature measurement: %d\n", ret);
        return ret;
    }

    // Poll DIE_TEMP_RDY (best practice from datasheet)
    do {
        msleep(10);
        ret = max30102_read_reg(data, MAX30102_REG_INTERRUPT_STATUS_2, &status, 1);
        if (ret < 0) return ret;
        timeout--;
    } while (!(status & (1 << MAX30102_INT_DIE_TEMP_RDY)) && timeout > 0);

    if (timeout <= 0) {
        dev_err(&data->client->dev, "Temperature measurement timeout\n");
        return -ETIMEDOUT;
    }

    ret = max30102_read_reg(data, MAX30102_REG_DIE_TEMP_INTEGER, &temp_int, 1);
    if (ret < 0) {
        dev_err(&data->client->dev, "Failed to read temperature integer: %d\n", ret);
        return ret;
    }

    ret = max30102_read_reg(data, MAX30102_REG_DIE_TEMP_FRACTION, &temp_frac, 1);
    if (ret < 0) {
        dev_err(&data->client->dev, "Failed to read temperature fraction: %d\n", ret);
        return ret;
    }

    *temp = (int8_t)temp_int + (temp_frac * 0.0625);  // Handle signed integer as per datasheet
    return 0;
}