**Maze Game for the STM32F429 board**

The game is an adaptation of the classic labyrinth ball game, where the player manuevers a ball around a maze containing holes on a physical wooden board. This program puts the game on the STM32F429 board, allowing the player to control the movement of the ball with using the boards gyroscope, outputting to the board's LCD display.

The maze is dynamically generated using the onboard hardware RNG generation

Ball movement is physics based using the board's gyroscope and simulated acceleration due to gravity.

LEDs are used to convey information to the player, allowing the player to use the board's button to temporarily make the ball invincible and output the status of this ability.

Ball physics, game timer, invincibility, and LED states run in seperate RTOS threads.

The game is won when the ball passes through all the waypoints, and the player loses when they leave the bounds of the maze or the ball enters a hole.

**Config**  
In the ApplicationCode.C file, different aspects of the game can be configured by the user before compilation.

**More Info**  
The program can be run in the STM32 IDE with using the physical board to play the game. All the files needed for this are included in the downloadable .zip. Otherwise, the main game files can be inspected inside the Core directory.
