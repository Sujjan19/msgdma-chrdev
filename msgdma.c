#include "msgdma.h"

/* Global driver data */
static struct msgdma_global msgdma_global = {
    .next_minor = 0,
    .device_list = LIST_HEAD_INIT(msgdma_global.device_list),
    .device_list_mutex = __MUTEX_INITIALIZER(msgdma_global.device_list_mutex),
};

/* Utility functions */
static void setbit_reg32(volatile void __iomem *reg, u32 mask)
{
    u32 val = ioread32(reg);
    iowrite32(val | mask, reg);
}

static void clearbit_reg32(volatile void __iomem *reg, u32 mask)
{
    u32 val = ioread32(reg);
    iowrite32((val & (~mask)), reg);
}

/* Dump detailed CSR/descriptor state for debugging timeouts */
static void dump_debug_state(struct msgdma_data *data)
{
    u32 csr_status = ioread32(&data->csr_reg->csr_status);
    u32 csr_ctrl = ioread32(&data->csr_reg->csr_ctrl);
    u32 fill_lvl = 0;
    u32 resp_fill = 0;
    void __iomem *desc = data->desc_reg;
    u32 d0 = 0, d1 = 0, d2 = 0, d3 = 0;

    /* Some platforms may not have these registers; read guarded if present */
    /* Many implementations provide fill level registers at known offsets */
    /* If your header defines them, replace below with the correct field names */
    /* Attempt to read fill levels if mapped within csr_reg structure */
    /* If they don't exist, these reads will return something harmless */
    /* (we already ioremap csr_reg and desc_reg earlier in probe). */

    /* Try reading fill level / resp fill level if present */
    /* (adjust names if your struct differs) */
    fill_lvl = ioread32(&data->csr_reg->csr_fill_lvl);
    resp_fill = ioread32(&data->csr_reg->csr_resp_fill_lvl);

    /* Read the descriptor words we wrote */
    d0 = ioread32(desc + 0x00);
    d1 = ioread32(desc + 0x04);
    d2 = ioread32(desc + 0x08);
    d3 = ioread32(desc + 0x0C);

    dev_err(data->dev, "MSGDMA-DBG: csr_status=0x%08x csr_ctrl=0x%08x\n",
            csr_status, csr_ctrl);
    dev_err(data->dev, "MSGDMA-DBG: fill_lvl=%u resp_fill_lvl=%u\n",
            fill_lvl, resp_fill);
    dev_err(data->dev, "MSGDMA-DBG: desc[0]=0x%08x desc[1]=0x%08x desc[2]=0x%08x desc[3]=0x%08x\n",
            d0, d1, d2, d3);
}

static void
msgdma_reset(struct msgdma_reg __iomem *reg)
{
    setbit_reg32(&reg->csr_ctrl, RESET_DISPATCHER);
    while (ioread32(&reg->csr_status) & RESETTING)
        ;
    clearbit_reg32(&reg->csr_ctrl, RESET_DISPATCHER);
}

/* Write a descriptor to the descriptor slave.
 * NOTE: descriptor slave registers are WRITE-ONLY (per TRM).
 * Do NOT attempt to read them back to verify contents.
 * Also: when in MM2S mode, do NOT write the write_addr field;
 * when in S2MM mode, do NOT write the read_addr field.
 */
static void
msgdma_push_descr(
    struct msgdma_reg __iomem *csr_reg,
    void __iomem *desc_reg,
    enum dma_direction dir,
    dma_addr_t rd_addr,
    dma_addr_t wr_addr,
    u32 len,
    u32 ctrl)
{
    /* For MM2S (memory -> stream): write read_addr, len, ctrl (do NOT write write_addr) */
    if (dir == MSGDMA_MM2S)
    {
        iowrite32((u32)rd_addr, desc_reg + 0x00); /* desc_read_addr */
        /* Do NOT write desc_write_addr for MM2S per TRM */
        iowrite32(len, desc_reg + 0x08);       /* desc_len */
        iowrite32(ctrl | GO, desc_reg + 0x0C); /* desc_ctrl (commit last) */
    }
    else
    {/* MSGDMA_S2MM: stream -> memory: write write_addr, len, ctrl (do NOT write read_addr) */
        iowrite32((u32)wr_addr, desc_reg + 0x04); /* desc_write_addr */
        /* Do NOT write desc_read_addr for S2MM per TRM */
        iowrite32(len, desc_reg + 0x08);       /* desc_len */
        iowrite32(ctrl | GO, desc_reg + 0x0C); /* desc_ctrl (commit last) */
    }

    /* Force posted writes to complete so HW sees them (read CSR as barrier) */
    (void)ioread32(&csr_reg->csr_status);

}

static int
msgdma_open(struct inode *node, struct file *f)
{
    struct msgdma_data *data;

    data = container_of(node->i_cdev, struct msgdma_data, cdev);
    f->private_data = data;

    return 0;
}

static int
msgdma_release(struct inode *node, struct file *f)
{
    return 0;
}

static ssize_t
msgdma_write(struct file *f, const char __user *ubuf, size_t len, loff_t *off)
{
    struct msgdma_data *data;
    dma_addr_t dma_addr;
    void *buf;
    size_t to_write;
    ssize_t write_ret;
    int ret;

    data = (struct msgdma_data *)f->private_data;

    /* Check if this device supports write operations */
    if (data->direction != MSGDMA_MM2S)
    {
        dev_err(data->dev, "Write operation not supported on this DMA direction\n");
        return -EPERM;
    }

    write_ret = len > DMA_BUF_SIZE ? DMA_BUF_SIZE : len;
    to_write = write_ret;

    /* Make transfer to DMA, in a pipeline fashion */
    data->transfer_in_progress = 1;
    dma_addr = data->dma_buf_handle;
    buf = data->dma_buf;

    /* Copy data from user space to DMA buffer */
    if (copy_from_user(data->dma_buf, ubuf, write_ret) != 0)
        return -EFAULT;

    while (to_write > MSGDMA_MAX_TX_LEN)
    {
        msgdma_push_descr(
            data->csr_reg,
            data->desc_reg,
            MSGDMA_MM2S,
            dma_addr, /* Source: DMA buffer */
            0,        /* Destination: Stream (address 0) */
            MSGDMA_MAX_TX_LEN,
            0);

        dma_addr += MSGDMA_MAX_TX_LEN;
        to_write -= MSGDMA_MAX_TX_LEN;
    }

    /* Last descriptor has to generate an IRQ */
    msgdma_push_descr(
        data->csr_reg,
        data->desc_reg,
        MSGDMA_MM2S,
        dma_addr, /* Source: DMA buffer */
        0,        /* Destination: Stream (address 0) */
        to_write,
        TX_COMPLETE_IRQ_EN);

    // dev_info(data->dev, "Starting write transfer: size=%zu, dma_addr=0x%llx\n",
    //          write_ret, (unsigned long long)data->dma_buf_handle);

    /* Wait for the transfer to complete */
    ret = wait_event_interruptible_timeout(
        data->complete_wq,
        !data->transfer_in_progress,
        TX_TIMEOUT);

    if (ret < 0)
        return -ERESTARTSYS;
    if (ret == 0)
    {
        dev_err(data->dev, "Write transfer timeout - checking status\n");
        dev_err(data->dev, "CSR Status: 0x%08x, CSR Ctrl: 0x%08x\n",
                ioread32(&data->csr_reg->csr_status),
                ioread32(&data->csr_reg->csr_ctrl));
        /* Dump detailed debug state (CSR, fill levels, and descriptor words) */
        dump_debug_state(data);
        return -EIO; // We have a timeout
    }

    return write_ret;
}

static ssize_t
msgdma_read(struct file *f, char __user *ubuf, size_t len, loff_t *off)
{
    struct msgdma_data *data;
    dma_addr_t rd_addr;
    size_t to_read;
    ssize_t read_ret;
    int ret;

    data = (struct msgdma_data *)f->private_data;

    /* Check if this device supports read operations */
    if (data->direction != MSGDMA_S2MM)
    {
        dev_err(data->dev, "Read operation not supported on this DMA direction\n");
        return -EPERM;
    }

    read_ret = len > DMA_BUF_SIZE ? DMA_BUF_SIZE : len;
    to_read = read_ret;

    /* Initiate transfer */
    rd_addr = data->dma_buf_handle;
    data->transfer_in_progress = 1;
    while (to_read > MSGDMA_MAX_TX_LEN)
    {
        msgdma_push_descr(
            data->csr_reg,
            data->desc_reg,
            MSGDMA_S2MM,
            0,       /* Source: Stream (address 0) */
            rd_addr, /* Destination: DMA buffer */
            MSGDMA_MAX_TX_LEN,
            0);

        to_read -= MSGDMA_MAX_TX_LEN;
        rd_addr += MSGDMA_MAX_TX_LEN;
    }

    /* Last descriptor sends an IRQ */
    msgdma_push_descr(
        data->csr_reg,
        data->desc_reg,
        MSGDMA_S2MM,
        0,       /* Source: Stream (address 0) */
        rd_addr, /* Destination: DMA buffer */
        to_read,
        TX_COMPLETE_IRQ_EN);

    // dev_info(data->dev, "Starting read transfer: size=%zu, dma_addr=0x%llx\n",
    //          read_ret, (unsigned long long)data->dma_buf_handle);

    /* Wait for transmission to complete */
    ret = wait_event_interruptible_timeout(
        data->complete_wq,
        !data->transfer_in_progress,
        TX_TIMEOUT);

    if (ret < 0)
        return -ERESTARTSYS;
    if (ret == 0)
    {
        dev_err(data->dev, "Read transfer timeout - checking status\n");
        dev_err(data->dev, "CSR Status: 0x%08x, CSR Ctrl: 0x%08x\n",
                ioread32(&data->csr_reg->csr_status),
                ioread32(&data->csr_reg->csr_ctrl));
        return -EIO; // We have a timeout
    }

    if (copy_to_user(ubuf, data->dma_buf, read_ret) != 0)
        return -EFAULT;

    return read_ret;
}

static irqreturn_t
msgdma_irq_handler(int irq, void *dev_id)
{
    struct msgdma_data *data = (struct msgdma_data *)dev_id;
    struct msgdma_reg __iomem *csr_reg = data->csr_reg;
    u32 status;

    status = ioread32(&csr_reg->csr_status);

    /* Check if this interrupt is for us */
    if (status & IRQ)
    {
        /* Clear IRQ by writing 1 to the IRQ bit */
        iowrite32(IRQ, &csr_reg->csr_status);

        data->transfer_in_progress = 0;
        wake_up_interruptible(&data->complete_wq);

        // dev_info(data->dev, "IRQ handled: status=0x%08x\n", status);
        return IRQ_HANDLED;
    }

    return IRQ_NONE;
}

static int
msgdma_parse_dt(struct platform_device *pdev, struct msgdma_data *data)
{
    struct device *dev = &pdev->dev;
    struct resource *res;
    const char *direction_str;
    int ret;

    /* Parse DMA direction */
    ret = of_property_read_string(dev->of_node, "dma-direction", &direction_str);
    if (ret)
    {
        dev_err(dev, "Missing 'dma-direction' property\n");
        return ret;
    }

    if (strcmp(direction_str, "mm2s") == 0)
    {
        data->direction = MSGDMA_MM2S;
    }
    else if (strcmp(direction_str, "s2mm") == 0)
    {
        data->direction = MSGDMA_S2MM;
    }
    else
    {
        dev_err(dev, "Invalid dma-direction '%s'. Must be 'mm2s' or 's2mm'\n",
                direction_str);
        return -EINVAL;
    }

    /* Get CSR register region */
    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (!res)
    {
        dev_err(dev, "CSR register resource not defined\n");
        return -ENODEV;
    }

    data->csr_reg = devm_ioremap_resource(dev, res);
    if (IS_ERR(data->csr_reg))
    {
        dev_err(dev, "Could not remap CSR registers\n");
        return PTR_ERR(data->csr_reg);
    }

    /* Get descriptor slave register region */
    res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
    if (!res)
    {
        dev_err(dev, "Descriptor register resource not defined\n");
        return -ENODEV;
    }

    data->desc_reg = devm_ioremap_resource(dev, res);
    if (IS_ERR(data->desc_reg))
    {
        dev_err(dev, "Could not remap descriptor registers\n");
        return PTR_ERR(data->desc_reg);
    }

    /* Get interrupt */
    data->irq = platform_get_irq(pdev, 0);
    if (data->irq < 0)
    {
        dev_err(dev, "Could not get IRQ number\n");
        return data->irq;
    }

    return 0;
}

static int
msgdma_register_chrdev(struct msgdma_data *data)
{
    int ret = 0;
    char dev_name[32];

    mutex_lock(&msgdma_global.device_list_mutex);

    /* Assign minor number */
    data->minor = msgdma_global.next_minor;
    if (data->minor >= MAX_MSGDMA_DEVICES)
    {
        dev_err(data->dev, "Too many msgdma devices\n");
        ret = -ENOSPC;
        goto unlock;
    }

    data->dev_id = MKDEV(MAJOR(msgdma_global.dev_base), data->minor);

    /* Initialize and add character device */
    cdev_init(&data->cdev, &msgdma_fops);
    data->cdev.owner = THIS_MODULE;

    ret = cdev_add(&data->cdev, data->dev_id, 1);
    if (ret < 0)
    {
        dev_err(data->dev, "Character device add failed\n");
        goto unlock;
    }

    /* Create device node */
    snprintf(dev_name, sizeof(dev_name), "%s%d", DEV_NAME, data->minor);
    data->device = device_create(msgdma_global.class, data->dev,
                                 data->dev_id, data, dev_name);
    if (IS_ERR(data->device))
    {
        dev_err(data->dev, "Device creation failed\n");
        ret = PTR_ERR(data->device);
        goto cdev_del;
    }

    msgdma_global.next_minor++;
    list_add_tail(&data->list, &msgdma_global.device_list);

    dev_info(data->dev, "Created character device %s (direction: %s)\n",
             dev_name, (data->direction == MSGDMA_MM2S) ? "mm2s" : "s2mm");

    mutex_unlock(&msgdma_global.device_list_mutex);
    return 0;

cdev_del:
    cdev_del(&data->cdev);
unlock:
    mutex_unlock(&msgdma_global.device_list_mutex);
    return ret;
}

static void
msgdma_unregister_chrdev(struct msgdma_data *data)
{
    mutex_lock(&msgdma_global.device_list_mutex);

    device_destroy(msgdma_global.class, data->dev_id);
    cdev_del(&data->cdev);
    list_del(&data->list);

    mutex_unlock(&msgdma_global.device_list_mutex);
}

static int
msgdma_probe(struct platform_device *pdev)
{
    struct msgdma_data *data;
    struct device *dev = &pdev->dev;
    int ret = 0;

    data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
    if (!data)
        return -ENOMEM;

    data->dev = dev;
    platform_set_drvdata(pdev, data);
    INIT_LIST_HEAD(&data->list);

    /* Parse device tree properties */
    ret = msgdma_parse_dt(pdev, data);
    if (ret)
        return ret;

    /* Set up DMA */
    ret = dma_set_coherent_mask(dev, DMA_BIT_MASK(32));
    if (ret)
    {
        dev_err(dev, "Failed to set DMA coherent mask\n");
        return ret;
    }

    /* Allocate DMA buffer */
    data->dma_buf = dma_alloc_coherent(dev, DMA_BUF_SIZE,
                                       &data->dma_buf_handle, GFP_KERNEL);
    if (!data->dma_buf)
    {
        dev_err(dev, "Failed to allocate DMA buffer\n");
        return -ENOMEM;
    }

    /* Initialize the DMA hardware */
    msgdma_reset(data->csr_reg);
    setbit_reg32(&data->csr_reg->csr_ctrl,
        STOP_ON_EARLY_TERM | STOP_ON_ERROR | GLOBAL_INT_EN_MASK);
    /*
     * For debugging: enable global interrupts only.
     * Temporarily avoid STOP_ON_ERROR and STOP_ON_EARLY_TERM so the core
     * doesn't immediately halt while we're trying to observe behaviour.
     * Re-enable STOP_ON_ERROR / STOP_ON_EARLY_TERM after debugging.
     */
    // setbit_reg32(&data->csr_reg->csr_ctrl, GLOBAL_INT_EN_MASK);

    /* Request IRQ */
    ret = devm_request_irq(dev, data->irq, msgdma_irq_handler,
                           IRQF_SHARED, dev_name(dev), data);
    if (ret < 0)
    {
        dev_err(dev, "Could not request IRQ %d\n", data->irq);
        goto free_dma;
    }

    /* Initialize wait queue and status */
    init_waitqueue_head(&data->complete_wq);
    data->transfer_in_progress = 0;

    /* Register character device */
    ret = msgdma_register_chrdev(data);
    if (ret < 0)
        goto free_dma;

    dev_info(dev, "MSGDMA device probed successfully (direction: %s, irq: %d)\n",
             (data->direction == MSGDMA_MM2S) ? "mm2s" : "s2mm", data->irq);

    return 0;

free_dma:
    dma_free_coherent(dev, DMA_BUF_SIZE, data->dma_buf, data->dma_buf_handle);
    return ret;
}

static int
msgdma_remove(struct platform_device *pdev)
{
    struct msgdma_data *data = platform_get_drvdata(pdev);

    if (!data)
        return 0;

    msgdma_unregister_chrdev(data);

    /* Free DMA buffer */
    if (data->dma_buf)
    {
        dma_free_coherent(&pdev->dev, DMA_BUF_SIZE,
                          data->dma_buf, data->dma_buf_handle);
    }

    dev_info(&pdev->dev, "MSGDMA device removed\n");
    return 0;
}

static int __init
msgdma_init(void)
{
    int ret;

    /* Allocate character device region */
    ret = alloc_chrdev_region(&msgdma_global.dev_base, 0,
                              MAX_MSGDMA_DEVICES, DEV_NAME);
    if (ret < 0)
    {
        pr_err("msgdma: Failed to allocate character device region\n");
        return ret;
    }

    /* Create device class */
    msgdma_global.class = class_create(DEV_NAME);
    if (IS_ERR(msgdma_global.class))
    {
        pr_err("msgdma: Failed to create device class\n");
        ret = PTR_ERR(msgdma_global.class);
        goto unregister_chrdev;
    }

    /* Register platform driver */
    ret = platform_driver_register(&msgdma_driver);
    if (ret < 0)
    {
        pr_err("msgdma: Failed to register platform driver\n");
        goto destroy_class;
    }

    pr_info("msgdma: Driver initialized successfully\n");
    return 0;

destroy_class:
    class_destroy(msgdma_global.class);
unregister_chrdev:
    unregister_chrdev_region(msgdma_global.dev_base, MAX_MSGDMA_DEVICES);
    return ret;
}

static void __exit
msgdma_exit(void)
{
    platform_driver_unregister(&msgdma_driver);
    class_destroy(msgdma_global.class);
    unregister_chrdev_region(msgdma_global.dev_base, MAX_MSGDMA_DEVICES);
    pr_info("msgdma: Driver exited\n");
}

subsys_initcall(msgdma_init);
module_exit(msgdma_exit);

MODULE_DESCRIPTION("msgdma-chrdev - platform character device driver for msgdma IPcore");
MODULE_AUTHOR("Sujan SM");
MODULE_VERSION("2.0");
MODULE_LICENSE("GPL v2");
