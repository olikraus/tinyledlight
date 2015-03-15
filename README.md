Schematic and software for a PWM LED Light controller, based on ATTiny 85

# V2 #

I have started to work on version 2 of the LED controller.
https://github.com/olikraus/tinyledlight/blob/master/v2/tiny_led_light_v2_bw_100.png
![http://github.com/olikraus/tinyledlight/blob/master/v2/tiny_led_light_v2_bw_100.png](http://github.com/olikraus/tinyledlight/blob/master/v2/tiny_led_light_v2_bw_100.png)

State Machine

![http://github.com/olikraus/tinyledlight/blob/master/v2/tiny_led_light_v2_state_machine_700px.png](http://github.com/olikraus/tinyledlight/blob/master/v2/tiny_led_light_v2_state_machine_700px.png)


# V1 #

PWM LED Light controller, version 1
  * 14 Bit PWM resolution
  * Auto light off after 1 hour (adjustable in the source code)
  * Software [download](http://code.google.com/p/tinyledlight/downloads/list)

![http://github.com/olikraus/tinyledlight/blob/master/v1/tiny_led_light_bw_100.png](http://github.com/olikraus/tinyledlight/blob/master/v1/tiny_led_light_bw_100.png)

  * Use CNY17-4
  * `R1` can be 10K, 50K or 100K, use linear variant.
  * 5V is generated from 12V (or higher)
  * 12V (or higher) must be provided
  * 12V external power must not exceed the upper limit for the CNY17 and the maximum gate source voltage of the mosfet (20V)
  * For any voltage higher than 12V: Do not overheat the 7805 voltage regulator
  * LED must be connected to JP2
  * 12V external must match the LED specification
  * IRF630 or IRF630N
  * Output current only limited by IRF630/IRF630N (expect heating problems with higher currents)
  * Tested with "goobay" LED strip flex 33 SMD warm white (200mA)
  * Please note that this is just an experimental spare time project. See also the BSD License for this project.
