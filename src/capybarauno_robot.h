/* Copyright (c) 2015, Lab Ro.Co.Co ( http://www.dis.uniroma1.it/~labrococo/ )
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


/** @copyright Lab Ro.Co.Co. 2015
 * 
 * @note BSD license (3-clause BSD, aka new BSD)
 * 
 * @author j. dichtl
 * 
 */


// H E A D E R S
//
//

// project headers
#include "odom/capybarauno_odom.h"
#include "joy/capybarauno_move.h"


/// @class a class that combines the config for the odometry and the move/joystick classes
class CapybaraunoConfig : public MoveConfig, public OdomConfig {
	public:
		void printParameters() {
			MoveConfig::printParameters();
			OdomConfig::printParameters();
		}
};


/// @class interface class to use the robot: combines odometry and movement into a single class.
class CapybaraunoRobot {
	public:
		/// @brief constructor, that passes the config to the sub-components
		CapybaraunoRobot( CapybaraunoConfig config ) : config_(config), odom_(config_), move_(config_), hb_counter_(0) {};
		
		/// @brief initializes the object. should only be called when the config has been fully populated
		void init() {
			// initializes the odometry and the move object, which sets up the serial communication with the microcontroller
			if( !move_.init() )
				exit( 0 );
			if( !odom_.init() )
				exit( 0 );
			
			// init heartbeat variables
			heartbeat_.beat = 1;
			heartbeat_packet_.id=Heartbeat_Payload_ID;
			heartbeat_packet_.seq=0;
			heartbeat_packet_.heartbeat=heartbeat_;
			beat_ = 0;
		}
		
		/// @brief returns the current odometry via references
		void getOdometry( double &x, double &y, double &theta ) {
			OdomPose2d pose = odom_.getPose();
			x = pose.x_;
			y = pose.y_;
			theta = pose.theta_;
		}
		
		/// @brief sends a 'speed' command to the microcontroller. the parameter tv is the translational velocity, rv is the rotation speed
		void setSpeed( double &tv, double &rv ) {
			move_.sendSpeedCmd( tv*1000, rv*1000 );
		}
		
		/// @brief sends a heartbeat on every 50th call of this function. wihtout the heartbeat, the robot will stop about every second.
		void sendHeartbeat(){
			if( hb_counter_%50==0 ) {
				heartbeat_packet_.seq++;
				char heartbuff[255];
				char* pEnd=Packet_write( &heartbeat_packet_, heartbuff, config_.OdomConfig::comm_ascii_ );
				sendToUart( odom_.serial_fd_, heartbuff, pEnd-heartbuff, 0 );
				const char heart_1[] = "\xe2\x99\xa1";
				const char heart_2[] = "\xe2\x99\xa5";
				if( beat_ ) {
					printf( "%s\n", heart_1 );
					beat_=0;
				} else {
					printf( "%s\n", heart_2 );
					beat_=1;
				}

			}
			hb_counter_++;
		}
		
		/// @brief performs periodic tasks: querying the odometry (to empty the serial buffer) and sends a heartbeat signal.
		void spinOnce() {
			odom_.spinOnce();
			sendHeartbeat();
		}
		
		/// @brief calls spinOnce in a loop
		void spin() {
			while( true ) {
				spinOnce();
			}
		}
	protected:
		/// @brief object holding the config for this class
		CapybaraunoConfig config_;
		/// @brief object to handle the odometry
		EncoderOdom odom_;
		/// @brief object to send movement commands to the robot
		CapybaraunoMove move_;
		
		unsigned int hb_counter_;
		int beat_;
		struct Packet heartbeat_packet_;
		struct Heartbeat_Payload heartbeat_;
};







