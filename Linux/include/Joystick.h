#ifndef Joystick_H
#define Joystick_H

/**
 * A joystick interface class.
 * Call init() to connect with the joystick.
 * Call update() to poll the joystick state.
 * The bool button[] array contains the button info (pressed or not)
 * and the float axis[] array contain axis info in the range [-1:1].
 */
class Joystick
{
   public:
		bool DEBUG_PRINT;

      static const int numOfAxes = 6; /**< Number of supported axes. */
      static const int numOfButtons = 32; /**< Number of supported buttons. */

      bool button[numOfButtons];
      float axis[numOfAxes];

      Joystick();
      ~Joystick();

      // Initialize the joystick connection.
      bool init();

      // Poll the joystick state.
      bool update();

   private:
      int jd;
};
#endif //Joystick_H
