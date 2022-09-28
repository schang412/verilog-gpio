// SPDX-License-Identifier: BSD-3-Clause

/*
 * Verilog 32-Bit AXIL GPIO Controller Driver
 *
 * Author: Spencer Chang
 *    spencer@sycee.xyz
 *
 */

#include <linux/gpio/driver.h>
#include <linux/clk.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>


#define AXIL32_VERILOG_GPIO_MAX_GPIO    32
#define AXIL32_VERILOG_GPIO_NAME        "axil32-verilog_gpio"

#define A32V_GPIO_ID                    0x294EC110
#define A32V_GPIO_REV                   0x00000100

#define A32V_GPIO_ID_OFFSET             0x00 // Interface ID Register
#define A32V_GPIO_REV_OFFSET            0x04 // Interface Revision Register
#define A32V_GPIO_PNT_OFFSET            0x08 // Interface Next Pointer Register

#define A32V_GPIO_RESETR_OFFSET         0x10 // Interface Reset Register
#define A32V_GPIO_RESET_VECTOR          0x0A // the value to write

#define A32V_GPIO_DDR_OFFSET            0x24 // GPIO Data Direction Control Register
#define A32V_GPIO_OUT_OFFSET            0x28 // GPIO Data Output Control Register
#define A32V_GPIO_IN_OFFSET             0x2C // GPIO Data Input Control Register

#define A32V_GPIO_INTRPT_RISING_TRIGGER_OFFSET      0x30 // GPIO Data Input Interrupt Rising Trigger Register
#define A32V_GPIO_INTRPT_FALLING_TRIGGER_OFFSET     0x30 // GPIO Data Input Interrupt Falling Trigger Register
#define A32V_GPIO_INTRPT_MASK_OFFSET                0x38 // GPIO Data Input Interrupt Mask Register
#define A32V_GPIO_INTRPT_STATUS_OFFSET              0x3C // GPIO Data Input Interrupt Status Register


struct axil32v_gpio {
    struct gpio_chip gc;
    void __iomem *regs;

    struct device *dev;

    struct clk *pclk;
};

static void a32v_gpio_irq_mask(struct irq_data *d)
{
    struct gpio_chip *chip = irq_data_get_irq_chip_data(d);
    struct axil32v_gpio *a32v_gpio = gpiochip_get_data(chip);
    u32 interrupt_mask;

    interrupt_mask = ioread32(a32v_gpio->regs + A32V_GPIO_INTRPT_MASK_OFFSET);
    interrupt_mask = interrupt_mask | BIT(d->hwirq);
    iowrite32(interrupt_mask, a32v_gpio->regs + A32V_GPIO_INTRPT_MASK_OFFSET);
}

static void a32v_gpio_irq_unmask(struct irq_data *d)
{
    struct gpio_chip *chip = irq_data_get_irq_chip_data(d);
    struct axil32v_gpio *a32v_gpio = gpiochip_get_data(chip);
    u32 interrupt_mask;

    interrupt_mask = ioread32(a32v_gpio->regs + A32V_GPIO_INTRPT_MASK_OFFSET);
    interrupt_mask = interrupt_mask & ~BIT(d->hwirq);
    iowrite32(interrupt_mask, a32v_gpio->regs + A32V_GPIO_INTRPT_MASK_OFFSET);
}

static int a32v_gpio_irq_set_type(struct irq_data *d, unsigned int type)
{
    struct gpio_chip *chip = irq_data_get_irq_chip_data(d);
    struct axil32v_gpio *a32v_gpio = gpiochip_get_data(chip);
    unsigned long flags;
    u32 redge, fedge;
    u32 mask = BIT(d->hwirq);
    int ret = 0;

    spin_lock_irqsave(&chip->bgpio_lock, flags);
    redge = ioread32(a32v_gpio->regs + A32V_GPIO_INTRPT_RISING_TRIGGER_OFFSET) & ~mask;
    fedge = ioread32(a32v_gpio->regs + A32V_GPIO_INTRPT_FALLING_TRIGGER_OFFSET) & ~mask;

    if (type == IRQ_TYPE_EDGE_RISING) {
        redge |= mask;
    } else if (type == IRQ_TYPE_EDGE_FALLING) {
        fedge |= mask;
    } else if (type == IRQ_TYPE_EDGE_BOTH) {
        redge |= mask;
        fedge |= mask;
    } else {
        ret = -EINVAL;
        goto err_irq_type;
    }

    iowrite32(redge, a32v_gpio->regs + A32V_GPIO_INTRPT_RISING_TRIGGER_OFFSET);
    iowrite32(fedge, a32v_gpio->regs + A32V_GPIO_INTRPT_FALLING_TRIGGER_OFFSET);

err_irq_type:
    spin_unlock_irqrestore(&chip->bgpio_lock, flags);
    return ret;
}

static void a32v_gpio_irq_handler(struct irq_desc *desc)
{
    struct gpio_chip *chip = irq_desc_get_handler_data(desc);
    struct axil32v_gpio *a32v_gpio = gpiochip_get_data(chip);
    struct irq_chip *irqchip = irq_desc_get_chip(desc);
    unsigned long status;
    int hwirq;

    chained_irq_enter(irqchip, desc);

    status = ioread32(a32v_gpio->regs + A32V_GPIO_INTRPT_STATUS_OFFSET) &
            ~ioread32(a32v_gpio->regs + A32V_GPIO_INTRPT_MASK_OFFSET);

    for_each_set_bit(hwirq, &status, chip->ngpio)
        generic_handle_irq(irq_find_mapping(chip->irq.domain, hwirq));

    chained_irq_exit(irqchip, desc);
}

static struct irq_chip a32v_gpio_irqchip = {
    .name = "a32v-gpio",
    .irq_mask = a32v_gpio_irq_mask,
    .irq_unmask = a32v_gpio_irq_unmask,
    .irq_set_type = a32v_gpio_irq_set_type
};


static int a32v_gpio_verify_idrev(struct axil32v_gpio *a32v_gpio)
{
    u32 idr;
    u32 revr;

    idr = ioread32(a32v_gpio->regs + A32V_GPIO_ID_OFFSET);
    if (idr != A32V_GPIO_ID) {
        dev_err(a32v_gpio->dev, "IP ID (%d) does not match expected ID (%d)", idr, A32V_GPIO_ID);
        return 1;
    }
    revr = ioread32(a32v_gpio->regs + A32V_GPIO_REV_OFFSET);
    if (revr != A32V_GPIO_REV) {
        dev_warn(a32v_gpio->dev, "IP Revision (%d) does not match driver revision (%d)", revr, A32V_GPIO_REV);
    } else {
        dev_info(a32v_gpio->dev, "IP ID and Revision matches driver.");
    }

    return 0;
}

static int a32v_gpio_probe(struct platform_device *pdev)
{
    struct axil32v_gpio *a32v_gpio;
    struct resource *res;
    int ret, irq;
    u32 num_gpios = 32;

    a32v_gpio = devm_kzalloc(&pdev->dev, sizeof(*a32v_gpio), GFP_KERNEL);
    if (!a32v_gpio)
        return -ENOMEM;
    a32v_gpio->dev = &pdev->dev;

    // get the base address of the IP
    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    a32v_gpio->regs = devm_ioremap_resource(&pdev->dev, res);
    if (IS_ERR(a32v_gpio->regs))
        return PTR_ERR(a32v_gpio->regs);

    of_property_read_u32(pdev->dev.of_node, "ngpios", &num_gpios);
    if (num_gpios > AXIL32_VERILOG_GPIO_MAX_GPIO) {
        dev_err(&pdev->dev, "ngpios must be less than or equal to 32\n");
        return -EINVAL;
    }

    ret = a32v_gpio_verify_idrev(a32v_gpio);
    if (ret) {
        dev_err(&pdev->dev, "stopping driver (unmatched ip/driver id)");
        return ret;
    }

    // create the generic gpio driver
    ret = bgpio_init(&a32v_gpio->gc, &pdev->dev, 4, // mmio register width = 4*8
                    a32v_gpio->regs + A32V_GPIO_IN_OFFSET,  // mmio address for reading values
                    a32v_gpio->regs + A32V_GPIO_OUT_OFFSET, // mmio address for writing values
                    NULL,
                    a32v_gpio->regs + A32V_GPIO_DDR_OFFSET, // mmio address for setting dirout (1 means output)
                    NULL,
                    BGPIOF_READ_OUTPUT_REG_SET); // flag that says that the output register stores the previous value.
    if (ret) {
        dev_err(&pdev->dev, "Failed to register generic gpio, %d\n", ret);
        return ret;
    }

    a32v_gpio->gc.label = dev_name(&pdev->dev);
    a32v_gpio->gc.ngpio = num_gpios;
    a32v_gpio->gc.parent = &pdev->dev;
    a32v_gpio->gc.base = -1;
    a32v_gpio->gc.owner = THIS_MODULE;

    a32v_gpio->pclk = devm_clk_get(&pdev->dev, NULL);
    if (IS_ERR(a32v_gpio->pclk)) {
        ret = PTR_ERR(a32v_gpio->pclk);
        dev_err(&pdev->dev, "Failed to retrieve peripheral clock, %d\n", ret);
        return ret;
    }

    ret = clk_prepare_enable(a32v_gpio->pclk);
    if (ret) {
        dev_err(&pdev->dev, "Failed to enable the peripheral clock, %d\n", ret);
        return ret;
    }

    irq = platform_get_irq(pdev, 0);
    if (irq >= 0) {
        struct gpio_irq_chip *girq;

        girq = &a32v_gpio->gc.irq;
        girq->chip = &a32v_gpio_irqchip;
        girq->parent_handler = a32v_gpio_irq_handler;
        girq->num_parents = 1;
        girq->parents = devm_kcalloc(&pdev->dev, 1,
                                     sizeof(*girq->parents),
                                     GFP_KERNEL);
        if (!girq->parents) {
            ret = -ENOMEM;
            goto err_disable_clk;
        }
        girq->parents[0] = irq;
        girq->default_type = IRQ_TYPE_NONE;
        girq->handler = handle_edge_irq;
    }

    ret = devm_gpiochip_add_data(&pdev->dev, &a32v_gpio->gc, a32v_gpio);
    if (ret < 0) {
        dev_err(&pdev->dev, "Could not register gpiochip, %d\n", ret);
        goto err_disable_clk;
    }

    platform_set_drvdata(pdev, a32v_gpio);
    return 0;

err_disable_clk:
    clk_disable_unprepare(a32v_gpio->pclk);
    return ret;
}

static int a32v_gpio_remove(struct platform_device *pdev)
{
    struct axil32v_gpio *a32v_gpio = platform_get_drvdata(pdev);

    iowrite32(A32V_GPIO_RESET_VECTOR, a32v_gpio->regs + A32V_GPIO_RESETR_OFFSET);
    clk_disable_unprepare(a32v_gpio->pclk);

    return 0;
}

static const struct of_device_id axil32v_gpio_of_match[] = {
    { .compatible = "axil32verilog,gpio-0.1.0", },
    {}
};
MODULE_DEVICE_TABLE(of, axil32v_gpio_of_match);

/* work with hotplug and coldplug */
MODULE_ALIAS("platform:" AXIL32_VERILOG_GPIO_NAME);

static struct platform_driver axil32v_gpio_driver = {
    .probe = a32v_gpio_probe,
    .remove = a32v_gpio_remove,
    .driver = {
        .name = AXIL32_VERILOG_GPIO_NAME,
        .of_match_table = axil32v_gpio_of_match,
    },
};
module_platform_driver(axil32v_gpio_driver);

MODULE_DESCRIPTION("32-Bit AXIL GPIO Driver");
MODULE_AUTHOR("Spencer Chang");
MODULE_LICENSE("Dual BSD/GPL");