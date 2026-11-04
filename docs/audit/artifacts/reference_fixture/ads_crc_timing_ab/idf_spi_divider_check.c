/*
 * Divider functions below: SPDX-FileCopyrightText: 2015-2026 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Apache-2.0
 * Test extracts the actual ESP-IDF 5.5.5 divider functions; no hardware-register write.
 */
#include <assert.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
typedef uint32_t spi_ll_clock_val_t;
typedef union { struct {uint32_t clkcnt_l:6,clkcnt_h:6,clkcnt_n:6,clkdiv_pre:4,reserved22:9,clk_equ_sysclk:1;}; uint32_t val;} clock_reg_t;
static struct {clock_reg_t clock;} GPSPI2;
static inline int spi_ll_freq_for_pre_n(int fapb, int pre, int n)
{
    return (fapb / (pre * n));
}
static inline int spi_ll_master_cal_clock(int fapb, int hz, int duty_cycle, spi_ll_clock_val_t *out_reg)
{
    typeof(GPSPI2.clock) reg = {.val = 0};
    int eff_clk;

    //In hw, n, h and l are 1-64, pre is 1-8K. Value written to register is one lower than used value.
    if (hz > ((fapb / 4) * 3)) {
        //Using Fapb directly will give us the best result here.
        reg.clkcnt_l = 0;
        reg.clkcnt_h = 0;
        reg.clkcnt_n = 0;
        reg.clkdiv_pre = 0;
        reg.clk_equ_sysclk = 1;
        eff_clk = fapb;
    } else {
        //For best duty cycle resolution, we want n to be as close to 32 as possible, but
        //we also need a pre/n combo that gets us as close as possible to the intended freq.
        //To do this, we bruteforce n and calculate the best pre to go along with that.
        //If there's a choice between pre/n combos that give the same result, use the one
        //with the higher n.
        int pre, n, h, l;
        int bestn = -1;
        int bestpre = -1;
        int besterr = 0;
        int errval;
        for (n = 2; n <= 64; n++) { //Start at 2: we need to be able to set h/l so we have at least one high and one low pulse.
            //Effectively, this does pre=round((fapb/n)/hz).
            pre = ((fapb / n) + (hz / 2)) / hz;
            if (pre <= 0) {
                pre = 1;
            }
            if (pre > 16) {
                pre = 16;
            }
            errval = abs(spi_ll_freq_for_pre_n(fapb, pre, n) - hz);
            if (bestn == -1 || errval <= besterr) {
                besterr = errval;
                bestn = n;
                bestpre = pre;
            }
        }

        n = bestn;
        pre = bestpre;
        l = n;
        //This effectively does round((duty_cycle*n)/256)
        h = (duty_cycle * n + 127) / 256;
        if (h <= 0) {
            h = 1;
        }

        reg.clk_equ_sysclk = 0;
        reg.clkcnt_n = n - 1;
        reg.clkdiv_pre = pre - 1;
        reg.clkcnt_h = h - 1;
        reg.clkcnt_l = l - 1;
        eff_clk = spi_ll_freq_for_pre_n(fapb, pre, n);
    }
    if (out_reg != NULL) {
        *(uint32_t *)out_reg = reg.val;
    }
    return eff_clk;
}
int main(void) {
 const int sources[]={40000000,80000000,160000000};
 for (unsigned i=0;i<3;i++) {
  clock_reg_t reg={0};
  int hz=spi_ll_master_cal_clock(sources[i],2000000,64,&reg.val);
  uint64_t highNs=(uint64_t)(reg.clkcnt_h+1)*(reg.clkdiv_pre+1)*1000000000u/sources[i];
  uint64_t periodNs=(uint64_t)(reg.clkcnt_n+1)*(reg.clkdiv_pre+1)*1000000000u/sources[i];
  assert(hz==2000000 && highNs==125u && periodNs-highNs==375u);
  assert(highNs>=60u && periodNs-highNs>=40u);
  printf("ADS_SPI_DIVIDER,sourceHz=%d,actualHz=%d,highNs=%llu,lowNs=%llu,duty=64,passed=1\n",sources[i],hz,(unsigned long long)highNs,(unsigned long long)(periodNs-highNs));
 }
}
