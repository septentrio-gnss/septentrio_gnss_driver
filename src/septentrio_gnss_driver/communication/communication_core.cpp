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

#include <septentrio_gnss_driver/communication/communication_core.hpp>

/**
 * @file communication_core.cpp
 * @date 22/08/20
 * @brief Highest-Level view on communication services
 */
 
io_comm_rx::Comm_IO::Comm_IO(): handlers_() {}

void io_comm_rx::Comm_IO::send(std::string cmd)
{
	// Determine byte size of cmd and hand over to send() method of manager_
	manager_.get()->send(cmd, cmd.size());
}

bool io_comm_rx::Comm_IO::initializeTCP(std::string host, std::string port)
{
	ROS_DEBUG("Calling initializeTCP() method..");
	host_ = host;
	port_ = port;
	// The io_context, of which io_service is a typedef of; it represents your program's link to the 
	// operating system's I/O services.
	boost::shared_ptr<boost::asio::io_service> io_service(new boost::asio::io_service);
	boost::asio::ip::tcp::resolver::iterator endpoint;

	try 
	{
		boost::asio::ip::tcp::resolver resolver(*io_service);
		// Note that tcp::resolver::query takes the host to resolve or the IP as the first parameter and 
		// the name of the service (as defined e.g. in /etc/services on Unix hosts) as second parameter. 
		// For the latter, one can also use a numeric service identifier (aka port number). In any case, 
		// it returns a list of possible endpoints, as there might be several entries for a single host.
		endpoint = resolver.resolve(boost::asio::ip::tcp::resolver::query(host, port)); // Resolves query object.. 
	} 
	catch (std::runtime_error& e) 
	{
		throw std::runtime_error("Could not resolve " + host + " on port " + port + ": " + e.what());
		return false;
	}

	boost::shared_ptr<boost::asio::ip::tcp::socket> socket(new boost::asio::ip::tcp::socket(*io_service));

	try 
	{
		// The list of endpoints obtained above may contain both IPv4 and IPv6 endpoints, so we need to 
		// try each of them until we find one that works. This keeps the client program independent of 
		// a specific IP version. The boost::asio::connect() function does this for us automatically. 
		socket->connect(*endpoint);
	} 
	catch (std::runtime_error& e) 
	{
		throw std::runtime_error("Could not connect to " + endpoint->host_name() + ": " + 
			endpoint->service_name() + ": " + e.what());
		return false;
	}

	ROS_INFO("Connected to %s: %s.", endpoint->host_name().c_str(), endpoint->service_name().c_str());

	if (manager_)
	{
		ROS_ERROR("You have called the InitializeTCP() method though an AsyncManager object is already available! Start all anew..");
		return false;
	}
	setManager(boost::shared_ptr<Manager>(new AsyncManager<boost::asio::ip::tcp::socket>(socket, io_service)));
	ROS_DEBUG("Leaving initializeTCP() method..");
	return true;
}

 
bool io_comm_rx::Comm_IO::initializeSerial(std::string port, uint32_t baudrate, std::string flowcontrol) 
{
	ROS_DEBUG("Calling initializeSerial() method..");
	serial_port_ = port;
	baudrate_ = baudrate;
	// The io_context, of which io_service is a typedef of; it represents your program's link to the 
	// operating system's I/O services.
	boost::shared_ptr<boost::asio::io_service> io_service(new boost::asio::io_service); 
	// To perform I/O operations the program needs an I/O object, here "serial".
	boost::shared_ptr<boost::asio::serial_port> serial(new boost::asio::serial_port(*io_service));

	// We attempt the opening of the serial port..
	try 
	{
		serial->open(serial_port_);
	} 
	catch (std::runtime_error& e) 
	{
		// and return an error message in case it fails.
		throw std::runtime_error("Could not open serial port : " + serial_port_ + ": " + e.what());
		return false;
	}

	ROS_INFO("Opened serial port %s", serial_port_.c_str());
    ROS_DEBUG("Our boost version is %u.", BOOST_VERSION);
	if(BOOST_VERSION < 106600) // E.g. for ROS melodic (i.e. Ubuntu 18.04), the version is 106501, standing for 1.65.1.
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
		
		// Hardware flow control settings..
		if (flowcontrol == "RTS|CTS") 
		{
			tio.c_iflag &= ~(IXOFF | IXON);
			tio.c_cflag |= CRTSCTS;
		} 
		else 
		{
			tio.c_iflag &= ~(IXOFF | IXON);
			tio.c_cflag &= ~CRTSCTS;
		}
		// Setting serial port to "raw" mode to prevent EOF exit..
		cfmakeraw(&tio);
		
		// Commit settings, syntax is 
		// int tcsetattr(int fd, int optional_actions, const struct termios *termios_p);
		tcsetattr(fd, TCSANOW, &tio);
	}

	// Set the I/O manager
	if (manager_) 
	{
		ROS_ERROR("You have called the initializeSerial() method though an AsyncManager object is already available! Start all anew..");
		return false;
	}
	ROS_DEBUG("Creating new Async-Manager object..");
	setManager(boost::shared_ptr<Manager>(new AsyncManager<boost::asio::serial_port>(serial, io_service)));
	
	// Setting the baudrate, incrementally..
	ROS_DEBUG("Gradually increasing the baudrate to the desired value...");
	boost::asio::serial_port_base::baud_rate current_baudrate;
	ROS_DEBUG("Initiated current_baudrate object...");
	try 
	{
		serial->get_option(current_baudrate); // Note that this sets current_baudrate.value() often to 115200, since by default, all Rx COM ports,
		// at least for mosaic Rxs, are set to a baudrate of 115200 baud, using 8 data-bits, no parity and 1 stop-bit.
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
        return false;
    }
	// Gradually increase the baudrate to the desired value
	// The desired baudrate can be lower or larger than the
	// current baudrate; the for loop takes care of both scenarios.
	ROS_DEBUG("Current baudrate is %u", current_baudrate.value());
	for (uint8_t i = 0; i < sizeof(BAUDRATES)/sizeof(BAUDRATES[0]); i++) 
	{
		if (current_baudrate.value() == baudrate_)
		{
			break; // Break if the desired baudrate has been reached.
		}
		if(current_baudrate.value() >= BAUDRATES[i] && baudrate_ > BAUDRATES[i])
		{
			continue; 
		}
		// Increment until Baudrate[i] matches current_baudrate.
		try 
		{
			serial->set_option(boost::asio::serial_port_base::baud_rate(BAUDRATES[i]));
		} catch(boost::system::system_error& e)
		{
			
			ROS_ERROR("set_option failed due to %s", e.what());
			ROS_INFO("Additional info about error is %s", boost::diagnostic_information(e).c_str());
			return false;
		}
		usleep(SET_BAUDRATE_SLEEP_);
		//boost::this_thread::sleep(boost::posix_time::milliseconds(SET_BAUDRATE_SLEEP_*1000)); 
		// Boost's sleep would yield an error message with exit code -7 the second time it is called, hence we use sleep() or usleep().
		try 
		{
			serial->get_option(current_baudrate);
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
			return false;
		}
		ROS_DEBUG("Set ASIO baudrate to %u", current_baudrate.value());
	}
	ROS_INFO("Set ASIO baudrate to %u, leaving InitializeSerial() method", current_baudrate.value());
	return true;
}

void io_comm_rx::Comm_IO::setManager(const boost::shared_ptr<Manager>& manager) 
{
	ROS_DEBUG("Called setManager() method");
	if (manager_) return; 
	manager_ = manager;
	manager_->setCallback(boost::bind(&CallbackHandlers::readCallback, &handlers_, _1, _2));
	ROS_DEBUG("Leaving setManager() method");
}

void io_comm_rx::Comm_IO::resetSerial(std::string port) 
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
		throw std::runtime_error("Could not open serial port :"
                             + serial_port_ + " " + e.what());
	}

	ROS_INFO("Reset serial port %s", serial_port_.c_str());

	// Sets the I/O worker
	if (manager_) return;
	setManager(boost::shared_ptr<Manager>(new AsyncManager<boost::asio::serial_port>(serial, io_service)));
	
	// Set the baudrate
	serial->set_option(boost::asio::serial_port_base::baud_rate(baudrate_));
}

