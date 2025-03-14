//*************************************************************************************
/** \file shares.h
 *    This file contains extern declarations for queues and other inter-task data
 *    communication objects used in a ME405/507/FreeRTOS project.
 *
 *  Revisions:
 *    \li 09-30-2012 JRR Original file was a one-file demonstration with two tasks
 *    \li 10-05-2012 JRR Split into multiple files, one for each task plus a main one
 *    \li 10-29-2012 JRR Reorganized with global queue and shared data references
 *
 *  License:
 *		This file is copyright 2012 by JR Ridgely and released under the Lesser GNU
 *		Public License, version 2. It intended for educational use only, but its use
 *		is not limited thereto. */
/*		THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *		AND	ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * 		IMPLIED 	WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * 		ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * 		LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUEN-
 * 		TIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * 		OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * 		CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * 		OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * 		OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */
//*************************************************************************************

// This define prevents this .h file from being included multiple times in a .cpp file
#ifndef _SHARES_H_
#define _SHARES_H_


//-------------------------------------------------------------------------------------
// Externs:  In this section, we declare variables and functions that are used in all
// (or at least two) of the files in the data acquisition project. Each of these items
// will also be declared exactly once, without the keyword 'extern', in one .cpp file
// as well as being declared extern here.


/**
 * \var print_ser_queue
 * \brief This queue allows tasks to send characters to the user interface task for display.
 */
extern frt_text_queue print_ser_queue;			// This queue allows tasks to send characters to the user interface task for display.

// Share Variables
extern shared_data<bool>* leftLimitSwitch;      // Left limit swtich
extern shared_data<bool>* rightLimitSwitch;     // Right limit switch

extern shared_data<int16_t>* pendulum_encoder;  // Pendulum encoder
extern shared_data<int16_t>* linear_position;	// Linear position of cart
extern shared_data<int16_t>* thMotor;			// Angular position of motor
extern shared_data<int16_t>* thdMotor;			// Angular velocity of motor
extern shared_data<int16_t>* linear_offset;     // Linear Offset for the carriage
extern shared_data<int16_t>* motor_command;

extern shared_data<bool>* begin; // case 0
extern shared_data<bool>* go; // case 2;
extern shared_data<bool>* stop; // case 3
extern shared_data<bool>* reset; // reset to idle

// Our attempt at a queue
//extern frt_queue<bool>* leftLimitSwitch;
//extern frt_queue<bool>* rightLimitSwitch;

//extern frt_queue<int16_t>* pendulum_encoder; // Value of the pendule encoder

#endif // _SHARES_H_
