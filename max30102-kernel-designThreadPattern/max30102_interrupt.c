#include <linux/workqueue.h>
#include "max30102.h"

/**
 * max30102_work_handler - Workqueue handler for interrupt processing
 * @work: Work structure
 */
void max30102_work_handler(struct work_struct *work)
{
    struct max30102_data *data = container_of(work, struct max30102_data, work);
    uint8_t status1 = 0, status2 = 0, write_ptr = 0, read_ptr = 0;
    uint8_t len = 0;
    uint8_t *fifo_data = NULL;
    int ret, i;

    if (!data) return;

    mutex_lock(&data->lock);

    ret = max30102_read_reg(data, MAX30102_REG_INTERRUPT_STATUS_1, &status1, 1);
    if (ret < 0) {
        dev_err(&data->client->dev, "Failed to read status1: %d\n", ret);
        goto unlock;
    }
    ret = max30102_read_reg(data, MAX30102_REG_INTERRUPT_STATUS_2, &status2, 1);
    if (ret < 0) {
        dev_err(&data->client->dev, "Failed to read status2: %d\n", ret);
        goto unlock;
    }

    // Clear status by reading (as per datasheet, status clears on read)

    if (status1 & (1 << MAX30102_INT_FIFO_FULL)) {
        ret = max30102_read_reg(data, MAX30102_REG_FIFO_WRITE_POINTER, &write_ptr, 1);
        if (ret < 0) {
            dev_err(&data->client->dev, "Failed to read write pointer: %d\n", ret);
            goto unlock;
        }
        ret = max30102_read_reg(data, MAX30102_REG_FIFO_READ_POINTER, &read_ptr, 1);
        if (ret < 0) {
            dev_err(&data->client->dev, "Failed to read read pointer: %d\n", ret);
            goto unlock;
        }

        len = (write_ptr - read_ptr + 32) % 32;  // Improved calculation from datasheet
        if (len == 0 || len > 32) {
            dev_err(&data->client->dev, "Invalid FIFO length: %d\n", len);
            goto unlock;
        }

        fifo_data = kmalloc(len * 6, GFP_KERNEL);
        if (!fifo_data) {
            dev_err(&data->client->dev, "Failed to allocate FIFO buffer\n");
            goto unlock;
        }

        ret = max30102_read_reg(data, MAX30102_REG_FIFO_DATA, fifo_data, len * 6);
        if (ret < 0) {
            dev_err(&data->client->dev, "Failed to read FIFO data: %d\n", ret);
            goto free_fifo;
        }

        for (i = 0; i < len; i++) {
            data->red_data[i] = (fifo_data[i*6] << 10) | (fifo_data[i*6+1] << 2) | (fifo_data[i*6+2] >> 6);  // 18-bit shift from datasheet
            data->ir_data[i] = (fifo_data[i*6+3] << 10) | (fifo_data[i*6+4] << 2) | (fifo_data[i*6+5] >> 6);
        }
        data->data_len = len;
        data->fifo_full = true;
        wake_up_interruptible(&data->wait_data_ready);  // Wake blocking read
        dev_info(&data->client->dev, "FIFO full: %d samples read\n", len);
    }

    if (status1 & (1 << MAX30102_INT_PPG_RDY))
        dev_info(&data->client->dev, "PPG ready interrupt\n");
    if (status1 & (1 << MAX30102_INT_ALC_OVF))
        dev_warn(&data->client->dev, "ALC overflow interrupt - adjust LED current\n");
    if (status1 & (1 << MAX30102_INT_PWR_RDY))
        dev_info(&data->client->dev, "Power ready interrupt\n");
    if (status2 & (1 << MAX30102_INT_DIE_TEMP_RDY))
        dev_info(&data->client->dev, "Die temperature ready interrupt\n");

free_fifo:
    kfree(fifo_data);
unlock:
    mutex_unlock(&data->lock);
}

/**
 * max30102_irq_handler - IRQ handler for MAX30102 interrupts
 * @irq: IRQ number
 * @dev_id: Device ID (max30102_data)
 * Returns: IRQ_HANDLED
 */
irqreturn_t max30102_irq_handler(int irq, void *dev_id)
{
    struct max30102_data *data = dev_id;
    if (!data) return IRQ_NONE;
    schedule_work(&data->work);
    return IRQ_HANDLED;
}