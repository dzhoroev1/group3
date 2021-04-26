#include <cstdio>
#include <iostream>
#include <limits>
#include <string>

#include <unistd.h>
#include <termios.h>
#include <poll.h>
#include <signal.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"

#include <sstream>

using namespace std;

static int terminal_descriptor = -1;
static struct termios terminal_original;
static struct termios terminal_settings;

const double BURGER_MAX_ANG_VEL = 2.84;

const double WAFFLE_MAX_LIN_VEL = 0.26;
const double WAFFLE_MAX_ANG_VEL = 1.82;

const double LIN_VEL_STEP_SIZE = 0.01;
const double ANG_VEL_STEP_SIZE = 0.1;



const string msg = "start";


/* Restore terminal to original settings */
void terminal_done() {
  if (terminal_descriptor != -1)
    tcsetattr(terminal_descriptor, TCSANOW, &terminal_original);
}
 

/* "Default" signal handler: restore terminal, then exit.  */
void terminal_signal(int signum) {
  cout << "Terminal_signal(" << signum << ")" << endl;
  if (terminal_descriptor != -1)
    tcsetattr(terminal_descriptor, TCSANOW, &terminal_original);
  /* exit() is not async-signal safe, but _exit() is.
   * Use the common idiom of 128 + signal number for signal exits.
   * Alternative approach is to reset the signal to default handler,
   * and immediately raise() it. */
  _exit(128 + signum);
}
 

/* 
 * Initialize terminal for non-canonical, non-echo mode,
 * that should be compatible with standard C I/O.
 * Returns 0 if success, nonzero errno otherwise.
*/
int terminal_init() {

  /* Already initialized? */
  if (terminal_descriptor != -1)
    return errno = 0;

  /* Which standard stream is connected to our TTY? */
  if (isatty(STDERR_FILENO))
    terminal_descriptor = STDERR_FILENO;
  else
  if (isatty(STDIN_FILENO))
    terminal_descriptor = STDIN_FILENO;
  else
  if (isatty(STDOUT_FILENO))
    terminal_descriptor = STDOUT_FILENO;
  else
    return errno = ENOTTY;

  /* Obtain terminal settings. */
  if (tcgetattr(terminal_descriptor, &terminal_original) ||
      tcgetattr(terminal_descriptor, &terminal_settings))
  {
    return errno = ENOTSUP;
  }

  // Disable buffering for terminal streams.
  if (isatty(STDIN_FILENO))
    setvbuf(stdin, NULL, _IONBF, 0);
  if (isatty(STDOUT_FILENO))
    setvbuf(stdout, NULL, _IONBF, 0);
  if (isatty(STDERR_FILENO))
    setvbuf(stderr, NULL, _IONBF, 0);

  /* At exit() or return from main(),
   * restore the original settings. */
  if (atexit(terminal_done))
    return errno = ENOTSUP;

  /* Set new "default" handlers for typical signals,
   * so that if this process is killed by a signal,
   * the terminal settings will still be restored first. */
  struct sigaction act;
  sigemptyset(&act.sa_mask);
  act.sa_handler = terminal_signal;
  act.sa_flags = 0;
  if (sigaction(SIGHUP,  &act, NULL) ||
      sigaction(SIGINT,  &act, NULL) ||
      sigaction(SIGQUIT, &act, NULL) ||
      sigaction(SIGTERM, &act, NULL) ||
#ifdef SIGXCPU
      sigaction(SIGXCPU, &act, NULL) ||
#endif
#ifdef SIGXFSZ    
      sigaction(SIGXFSZ, &act, NULL) ||
#endif
#ifdef SIGIO
      sigaction(SIGIO,   &act, NULL) ||
#endif
      sigaction(SIGPIPE, &act, NULL) ||
      sigaction(SIGALRM, &act, NULL))
  {
    return errno = ENOTSUP;
  }

  /* Let BREAK cause a SIGINT in input. */
  terminal_settings.c_iflag &= ~IGNBRK;
  terminal_settings.c_iflag |=  BRKINT;
 
  /* Ignore framing and parity errors in input. */
  terminal_settings.c_iflag |=  IGNPAR;
  terminal_settings.c_iflag &= ~PARMRK;
 
  /* Do not strip eighth bit on input. */
  terminal_settings.c_iflag &= ~ISTRIP;
 
  /* Do not do newline translation on input. */
  // terminal_settings.c_iflag &= ~(INLCR | IGNCR | ICRNL);
 
#ifdef IUCLC
  /* Do not do uppercase-to-lowercase mapping on input. */
  terminal_settings.c_iflag &= ~IUCLC;
#endif
 
  /* Use 8-bit characters. This too may affect standard streams,
   * but any sane C library can deal with 8-bit characters. */
  terminal_settings.c_cflag &= ~CSIZE;
  terminal_settings.c_cflag |=  CS8;

  /* Enable receiver. */
  terminal_settings.c_cflag |=  CREAD;

  /* Let INTR/QUIT/SUSP/DSUSP generate the corresponding signals. */
  terminal_settings.c_lflag |=  ISIG;

  /* Enable noncanonical mode.
   * This is the most important bit, as it disables line buffering etc. */
  terminal_settings.c_lflag &= ~ICANON;

  /* Disable echoing input characters. */
  terminal_settings.c_lflag &= ~(ECHO | ECHOE | ECHOK | ECHONL);

  /* Disable implementation-defined input processing. */
  terminal_settings.c_lflag &= ~IEXTEN;

  /* To maintain best compatibility with normal behaviour of terminals,
   * we set TIME=0 and MAX=1 in noncanonical mode. This means that
   * read() will block until at least one byte is available. */
  terminal_settings.c_cc[VTIME] = 0;
  terminal_settings.c_cc[VMIN] = 1;

  /* Done. */
  return errno = 0;
}


void set_terminal_raw_mode() {

  /* Set the new terminal settings.
   * Note that we don't actually check which ones were successfully
   * set and which not, because there isn't much we can do about it. */
  if (terminal_descriptor != -1)
    tcsetattr(terminal_descriptor, TCSANOW, &terminal_settings);
}


void set_terminal_original_mode() {
  /* Restore original terminal settings. */
  if (terminal_descriptor != -1)
    tcsetattr(terminal_descriptor, TCSANOW, &terminal_original);
}
 

bool wait_for_key_pressed(unsigned timeout_ms /* = 0 */) {
  if (terminal_descriptor == -1) return false;

  struct pollfd pls[ 1 ];
  pls[ 0 ].fd     = terminal_descriptor;
  pls[ 0 ].events = POLLIN | POLLPRI;

  return poll( pls, 1, timeout_ms ) > 0;
}


char getKeyStroke() {
  while(true) {
    char c = cin.get();
    if (c != 27) {
      return c;
    } else if (wait_for_key_pressed(50)) {
      c = cin.get();
      if (c == '[') {
        c = cin.get();
        switch(c) {
          case 'A':
            return 'A';
          case 'B':
            return 's';
          case 'C':
            return 'C';
          case 'D':
            return 'D';
          default:
            return 'Q';
        }
      } else {
        return 'Q';
      }
    } else {
      return 'Q';
    }
  }
}


double constrain(double input, double low, double high)
{
	if(input < low)
	{
		input = low;
	}
	else if(input > high)
	{
		input = high;
	}
	else
	{
		input = input;
	}

	return input;
}

void vels(double target_linear_vel, double target_angular_vel)
{
	cout << "currently:\tlinear vel" <<target_linear_vel<<"\t angular vel "<<target_angular_vel<<endl;
}

double makeSimpleProfile(double output, double input, double slop)
{
	if (input > output)
	{
        output = min( input, output + slop );
	}
    else if (input < output)
    {
        output = max( input, output - slop );
    }
    else
    {
        output = input;
    }

    return output;
}

double checkLinearLimitVelocity(double vel)
{
	vel = constrain(vel, -WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL);
	return vel;
}

double checkAngularLimitVelocity(double vel)
{
	 vel = constrain(vel, -WAFFLE_MAX_ANG_VEL, WAFFLE_MAX_ANG_VEL);
	 return vel;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "control");
	ros::NodeHandle n;

	ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  //ros::Rate loop_rate(10);
    int status = 0;
    double target_linear_vel   = 0.0;
    double target_angular_vel  = 0.0;
    double control_linear_vel  = 0.0;
    double control_angular_vel = 0.0;

 	terminal_init();
	set_terminal_raw_mode();
	//bool isContinue = true;
	while(ros::ok()) 
	{
		cout<<msg<<endl;
		while(true) 
		{
			if (wait_for_key_pressed(100)) break;
		}
		char c = getKeyStroke();
		switch(c) 
		{
			case 'w':
				target_linear_vel = checkLinearLimitVelocity(target_linear_vel + LIN_VEL_STEP_SIZE);
				status = status + 1;
				vels(target_linear_vel,target_angular_vel);	
				break;
			case 'x':
				target_linear_vel = checkLinearLimitVelocity(target_linear_vel - LIN_VEL_STEP_SIZE);
				status = status + 1;
				vels(target_linear_vel,target_angular_vel);
				break;
			case 'a':
				target_angular_vel = checkAngularLimitVelocity(target_angular_vel + ANG_VEL_STEP_SIZE);	
				status = status + 1;
				vels(target_linear_vel,target_angular_vel);
				break;
			case 'd':
				target_angular_vel = checkAngularLimitVelocity(target_angular_vel - ANG_VEL_STEP_SIZE);
				status = status + 1;
				vels(target_linear_vel,target_angular_vel);
				break;
			case 's':
				target_linear_vel   = 0.0;
                control_linear_vel  = 0.0;
                target_angular_vel  = 0.0;
                control_angular_vel = 0.0;	
                vels(target_linear_vel, target_angular_vel);
                break;
		}
		geometry_msgs::Twist msg;

		control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0));

        msg.linear.x = control_linear_vel; 
        msg.linear.y = 0.0;
        msg.linear.z = 0.0;

        control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0));
        msg.angular.x = 0.0; 
        msg.angular.y = 0.0; 
        msg.angular.z = control_angular_vel;

		vel_pub.publish(msg);

	}
	set_terminal_original_mode();
}