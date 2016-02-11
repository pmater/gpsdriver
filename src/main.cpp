#include "ros/ros.h"
#include <sensor_msgs/NavSatFix.h>
#include <termios.h>
#include <fcntl.h>
#include <iostream>
#include <string>
#include <boost/algorithm/string.hpp>
#include <signal.h>
#include <unistd.h> //Serial
#include <stdlib.h> //exit()

using namespace std;

int OpenSerial();
bool ReadGPSData(string &gpsMessage, string &checksum);
void ParseGPS(const string &gpsMessage, double &latitude, double &longitude, double &altitude);
bool ConfirmChecksum(const string &gpsMessage, const string &checksum);
void CloseSerialSig(int sig);

int tty_serial = -1;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "gpsdriver", ros::init_options::NoSigintHandler);
	signal(SIGINT, CloseSerialSig);
	ros::NodeHandle n;
	auto gpsPublisher = n.advertise<sensor_msgs::NavSatFix>("/gps/fix", 1000);

	//Open serial
	tty_serial = OpenSerial();
	if (tty_serial == -1)
	{
		cout << "Serial connection failed" << endl;
		return -1;
	}

	while (ros::ok())
	{
		bool gpsReadSuccess = false;
		string gpsMessage, checksum;
		while (!gpsReadSuccess)
		{
			gpsMessage.clear();
			checksum.clear();
			gpsReadSuccess = ReadGPSData(gpsMessage, checksum);
		}

		auto time = ros::Time::now();

		if (ConfirmChecksum(gpsMessage, checksum))
		{
			cout << "Checksum confirmed" << endl;
			double latitude, longitude, altitude;
			ParseGPS(gpsMessage, latitude, longitude, altitude);

			sensor_msgs::NavSatFix gpsROSMessage;
			gpsROSMessage.header.stamp = time;
			gpsROSMessage.latitude = latitude;
			gpsROSMessage.longitude = longitude;
			gpsROSMessage.altitude = altitude;
			gpsPublisher.publish(gpsROSMessage);
		}
		else
		{
			cout << "Checksum failed" << endl;
		}
		ros::spinOnce();
	}

	return 0;
}

int OpenSerial()
{
	int baudrate = 38400;

        int serial_fd = open("/dev/ttyUSB0",O_RDWR|O_NOCTTY);

        if (serial_fd < 0)
	{
   		cout << "Error in opening serial connection: " << serial_fd << endl;
                return -1;
        }

	struct termios laserfd;
        memset(&laserfd, 0, sizeof(laserfd));
        laserfd.c_iflag = 0;
        laserfd.c_oflag = 0;
        laserfd.c_cflag = CS8|CREAD|CLOCAL;
        laserfd.c_lflag = 0;
        laserfd.c_cc[VMIN] = 30;
        laserfd.c_cc[VTIME] = 2;

        fcntl(serial_fd, F_SETFL, 0);

        cfsetispeed(&laserfd, baudrate);
        cfsetospeed(&laserfd, baudrate);
        tcsetattr(serial_fd, TCSANOW, &laserfd);

	if (tcdrain(serial_fd) == 0)
	{
		cout << "TC drain succeeded" << endl;
        	return serial_fd;
	}
	else
	{
		cout << "TC drain failed" << endl;
		return -1;
	}
}

bool ReadGPSData(string &gpsMessage, string &checksum)
{
	uint8_t readValue = 0x00;

	// GGA data format
	cout << endl << "Searching for starting byte" << endl;
	while (readValue != '$')	// Find starting byte
	{
		read(tty_serial, &readValue, 1);
	}
	gpsMessage.push_back(readValue);
	cout << "Starting byte received" << endl;
	while (readValue != '*')	//Fill out data until we reach the delimiter
	{
		read(tty_serial, &readValue, 1);
		gpsMessage.push_back(readValue);

		if (gpsMessage.size() > 200)
		{
			cout << "Failed" << endl;
			return false;
		}
	}
	//Get checksum
        read(tty_serial, &readValue, 1);
        checksum.push_back(readValue);
        read(tty_serial, &readValue, 1);
        checksum.push_back(readValue);

	cout << gpsMessage << checksum << endl;
	return true;
}

void ParseGPS(const string &gpsMessage, double &latitude, double &longitude, double &altitude)
{
	vector<string> subStrings;
	boost::split(subStrings, gpsMessage, boost::is_any_of(","));

	latitude = stod(subStrings[2]);
	//NOTE: Sticking the -ve in front of the latitude only works in the southern hemisphere.
	//NMEA data format is DDmm.mmmmm, with D=degrees and m=minutes.
	//Want to convert to decimal degrees, like DD.DDDDDDDD.
	//First, move the decimal place to the left to get DD.mmmmmm. Cast to int to get DD.
	int latitudeDegrees = latitude / 100.0;
	//Subtract degrees from DD.mmmmm to get 0.mmmmm, then * 100 to get mm.mmm
	double latitudeDecimalPlaces = (latitude / 100.0 - latitudeDegrees) * 100;
	//Divide 0.mmmmm by 60 to get it in decimal units. Also make it negative because southern hemisphere.
	latitude = -(latitudeDegrees + latitudeDecimalPlaces / 60.0);

	longitude = stod(subStrings[4]);
	int longitudeDegrees = longitude / 100.0;
	double longitudeDecimalPlaces = (longitude / 100.0 - longitudeDegrees) * 100;
	longitude = longitudeDegrees + longitudeDecimalPlaces / 60.0;

	altitude = stod(subStrings[9]);
}

bool ConfirmChecksum(const string &gpsMessage, const string &checksum)
{
	cout << "Confirming checksum" << endl;
	uint8_t calcChecksum = 0x00;
	//Values we want to include in the checksum are between the $ and the *, non-inclusive
	for (int i = 1; i < gpsMessage.size() - 1; i++)
	{
		calcChecksum ^= gpsMessage[i];
		cout << gpsMessage[i];
	}

	static uint8_t _sendsum[2] = {0x00, 0x00};

	//Want to break the checksum into 2 bytes for easy comparison to message
	_sendsum[0] = ((calcChecksum & 0xf0) >> 4);
	_sendsum[1] = (calcChecksum & 0x0f);

	//I don't know what this does
	if (_sendsum[0] > 0x09)
	_sendsum[0] = _sendsum[0] + 0x37;
	else _sendsum[0] = _sendsum[0] + 0x30;

	if (_sendsum[1] > 0x09)
	_sendsum[1] = _sendsum[1] + 0x37;
	else _sendsum[1] = _sendsum[1] + 0x30;

	cout << endl << "Calculated: " << _sendsum[0] << _sendsum[1] << endl;
	return checksum[0] == _sendsum[0] && checksum[1] == _sendsum[1];
}

void CloseSerialSig(int sig)
{
	close(tty_serial);
	cout << "Serial connection closed" << endl;
	ros::shutdown();
	exit(EXIT_SUCCESS);
}
