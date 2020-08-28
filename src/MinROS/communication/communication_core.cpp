// *****************************************************************************
//
// © Copyright 2020, Septentrio NV/SA.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//    1. Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//    2. Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//    3. Neither the name of the copyright holder nor the names of its
//       contributors may be used to endorse or promote products derived
//       from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE. 
//
// *****************************************************************************

#include <MinROS/communication/communication_core.hpp>

/**
 * @file communication_core.cpp
 * @date 22/08/20
 * @brief Highest-Level view on communication services
 */
io_comm_mosaic::Comm_IO::Comm_IO(): handlers_() {}

template <typename T>
bool io_comm_mosaic::Comm_IO::poll(T& message, std::string message_ID, const boost::posix_time::time_duration& timeout) 
{
	if (!manager_) return false;
	return handlers_.poll(message, message_ID, timeout);
}

 
bool io_comm_mosaic::Comm_IO::InitializeSerial(std::string port, uint32_t baudrate,
			std::string flowcontrol) 
{
	//ROS_DEBUG("Current debug value after calling initializeserial() method is %u", io_comm_mosaic::debug);
	ROS_DEBUG("mosaic: Calling InitializeSerial method..");
	serial_port_ = port;
	baudrate_ = baudrate;
	boost::shared_ptr<boost::asio::io_service> io_service(
		new boost::asio::io_service); 
		// The io_context, of which io_service is a typedef of,
		// represents your program's link to the operating system's I/O services.
	boost::shared_ptr<boost::asio::serial_port> serial(
		new boost::asio::serial_port(*io_service));
		// To perform I/O operations the program needs an I/O object, here "serial".

	// We attempt the opening of the serial port..
	try {
		serial->open(serial_port_);
	} 
	catch (std::runtime_error& e) 
	{
	// and return an error message in case it fails.
		throw std::runtime_error("mosaic-X5: Could not open serial port :"
								 + serial_port_ + " " + e.what());
		return false;
	}

	ROS_INFO("mosaic-X5: Opened serial port %s", serial_port_.c_str());
    ROS_DEBUG("Our boost version is %u.", BOOST_VERSION);
	if(BOOST_VERSION < 106600) // e.g. for ROS melodic (i.e. Ubuntu 18.04) it is 106501, standing for 1.65.1
	{
		// Workaround to set some options for the port manually, 
		// cf. https://github.com/mavlink/mavros/pull/971/files
		// This function native_handle() may be used to obtain 
		// the underlying representation of the serial port.
		// Conversion from type native_handle_type to int is done implicitly.
		int fd = serial->native_handle();
		termios tio;
		// Get terminal attribute, follows the syntax
		// int tcgetattr(int fd, struct termios *termios_p);
		tcgetattr(fd, &tio); 
		
		// Setting hardware flow control settings..
		if (flowcontrol == "RTS|CTS") {
			tio.c_iflag &= ~(IXOFF | IXON);
			tio.c_cflag |= CRTSCTS;
		} else {
			tio.c_iflag &= ~(IXOFF | IXON);
			tio.c_cflag &= ~CRTSCTS;
		}
		// Setting serial port to "raw" mode to prevent EOF exit..
		cfmakeraw(&tio);
		
		// Commit settings, syntax is 
		// int tcsetattr(int fd, int optional_actions,
		// 		const struct termios *termios_p);
		tcsetattr(fd, TCSANOW, &tio);
	}

	// Set the I/O manager
	if (manager_) 
	{
		ROS_ERROR("You have called InitializeSerial twice! Start all anew or call ResetSerial..");
		return false;
	}
	ROS_DEBUG("Creating new Async-Manager..");
	setManager(boost::shared_ptr<Manager>(new AsyncManager<boost::asio::serial_port>(serial, io_service)));
	
	//ROS_DEBUG("Finished creating new Async-Manager, have not yet called its read_callback_, since that will only be populated by the readCallback method of the CallbackHandlers class momentarily..");

	// Setting the baudrate, incrementally..
	ROS_DEBUG("Gradually increasing the baudrate to the desired value...");
	boost::asio::serial_port_base::baud_rate current_baudrate;
	ROS_DEBUG("Initiated current_baudrate object...");
	try 
	{
		serial->get_option(current_baudrate); 	// Embed with throw to catch bad file descriptor error or similar ones..
												// Often sets current_baudrate.value() magically to 115200
	} catch(boost::system::system_error& e)
	{
		
		ROS_ERROR("get_option failed due to %s", e.what());
        ROS_INFO("Additional info about error is %s", boost::diagnostic_information(e).c_str());
		/*
		boost::system::error_code e_loop;
		do // Caution: Might cause infinite loop..
		{
			serial->get_option(current_baudrate, e_loop);
		} while(e_loop);
		*/
        return 1;
    }
	// Gradually increase the baudrate to the desired value
	// The desired baudrate can be lower or larger than the
	// current baudrate; the for loop takes care of both scenarios.
	ROS_DEBUG("Current baudrate is %u", current_baudrate.value());
	for (uint8_t i = 0; i < sizeof(Baudrates)/sizeof(Baudrates[0]); i++) 
	{
		if (current_baudrate.value() == baudrate_)
			break; 
			// Break if the desired baudrate has been reached
		if(current_baudrate.value() > Baudrates[i] && baudrate_ > Baudrates[i])
			continue; 
			// Increment until Baudrate[i] matches current_baudrate
		serial->set_option(boost::asio::serial_port_base::baud_rate(Baudrates[i]));
		boost::this_thread::sleep(boost::posix_time::milliseconds(SetBaudrateSleepMs));
		serial->get_option(current_baudrate);
		ROS_DEBUG("mosaic: Set ASIO baudrate to %u", current_baudrate.value());
	}
	ROS_INFO("mosaic: Set ASIO baudrate to %u, leaving InitializeSerial() method", current_baudrate.value());
	return true;
}

void io_comm_mosaic::Comm_IO::setManager(const boost::shared_ptr<Manager>& manager) {
	ROS_DEBUG("Entered setManager");
	if (manager_) return; 
	manager_ = manager;
	ROS_DEBUG("About to call setCallback");
	manager_->setCallback(boost::bind(&CallbackHandlers::readCallback, &handlers_, _1, _2));
}


/**
 * Needs to be checked..
 */
void io_comm_mosaic::Comm_IO::resetSerial(std::string port) 
{
	serial_port_ = port;
	boost::shared_ptr<boost::asio::io_service> io_service(new boost::asio::io_service);
	boost::shared_ptr<boost::asio::serial_port> serial(new boost::asio::serial_port(*io_service));

	// Try to open serial port
	try 
	{
		serial->open(serial_port_);
	} catch (std::runtime_error& e) 
	{
		throw std::runtime_error("mosaic: Could not open serial port :"
                             + serial_port_ + " " + e.what());
	}

	ROS_INFO("mosaic: Reset serial port %s", serial_port_.c_str());

	// Sets the I/O worker
	if (manager_) return;
	setManager(boost::shared_ptr<Manager>(new AsyncManager<boost::asio::serial_port>(serial, io_service)));

	/*
	// Polls ReceiverStatus block
	std::vector<uint8_t> receiverstatus;
	if (!poll(..., receiverstatus)) 
	{
		ROS_ERROR("Resetting Serial Port: Could not poll ReceiverStatus");
		return;
	}
	ReceiverStatus struct_object;
	if(!read(struct_object, ...default_timeout_)) 
	{
		ROS_ERROR("Resetting Serial Port: Could not read polled ReceiverStatus");
		return;
	}
	*/

	// Set the baudrate
	serial->set_option(boost::asio::serial_port_base::baud_rate(baudrate_));
}

