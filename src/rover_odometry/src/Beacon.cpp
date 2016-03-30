#include <Beacon/Beacon.h>
#include <iostream>
#include <limits>
using std::deque;
using std::numeric_limits;
using std::cout; 
using std::endl;
using std::pair; 
using std::vector;
using std::unordered_map; 
using std::string;


RoverBeacon::RoverBeacon(string _id) : id(_id){}

RoverBeacon::RoverBeacon(string _id, 
	vector<pair<int, sieveBeaconData>> sieveBeacons, 
		pair<double,double> roverOffset) : id(_id), 
			offset(roverOffset) {
	// loop through the sieveBeacon vector to intialize the rover
	// beacon;
	for(auto beacon: sieveBeacons){ 
		// store the id of the sieve beacon 
		// and its associated data
		beaconReadings.insert({beacon.first, beacon.second}); 
	}
}

double 
RoverBeacon::getReading(int _id){
	beaconIterator = beaconReadings.find(_id);
	return beaconIterator->second.reading;
}


void 
RoverBeacon::updateReading(int _id, double _reading){
	beaconIterator = beaconReadings.find(_id);
	beaconIterator->second.reading = _reading - 
				beaconIterator->second.bias;
	positionUpdated = false;

}

double 
RoverBeacon::getBias(int _id){
	beaconIterator = beaconReadings.find(_id);
	return beaconIterator->second.bias;
}

void 
RoverBeacon::updateBias(int _id, double _bias){
	//std::cout << "enter updateBias with following value: " << 
						//bias_ << endl;
	beaconIterator = beaconReadings.find(_id);
	beaconIterator->second.bias = _bias;
	//std::cout << "leaving updateBias" << endl;
}

pair<double,double>
RoverBeacon::getPosition(){
	//Position of the Beacon should only be updated 
	//if new readings have been taken
	if(!positionUpdated) {
		updatePosition();
		positionUpdated = true;
	}
	return position;
}

std::unordered_map<int, RoverBeacon::sieveBeaconData> RoverBeacon::getBeaconReadings () {
    return beaconReadings;
}

void
RoverBeacon::updatePosition(){
	vector<double> readings;
	vector<pair<double,double>> offsets;
	auto it = beaconReadings.begin();
	auto endIt = beaconReadings.end();
	for (; it != endIt; ++it) {
		readings.push_back(it->second.reading);
		offsets.push_back(it->second.offset);
	}
	position = steepest_descent(readings,offsets, position);

}

string
RoverBeacon::getId(){
	return id;
}

pair<double, double>
RoverBeacon::getOffset() {
    return offset;
}

double 
beaconError(double r_1, double h_1, double k_1, 
		const pair<double, double>& guess) {
    double error_1;
    double diff_x_1, diff_y_1; //edges of triangles
    double dist_1; //distance of guess from beacon
    diff_x_1 = guess.first - h_1; //bottom edge of triangle
    diff_y_1 = guess.second - k_1;
    dist_1 = sqrt(diff_x_1 * diff_x_1 + diff_y_1 * diff_y_1);// py theor
    error_1 = dist_1 - r_1;
    return error_1;
}


double 
beaconPartialY(double r_1, double h_1, double k_1, 
		const pair<double, double>& guess) {
    double diff_x_1, diff_y_1; //edges of triangles
    double dist_1;
    diff_x_1 = guess.first - h_1; //bottom edge of triangle
    diff_y_1 = guess.second - k_1;   
    dist_1 = sqrt(diff_x_1 * diff_x_1 + diff_y_1 * diff_y_1);// py theor
    double circle1Deriv = diff_y_1/dist_1;
    if (dist_1 < r_1) {
        circle1Deriv *= -1;
    }
    return circle1Deriv;
}


double 
beaconPartialX(double r_1, double h_1, double k_1, 
		const pair<double, double>& guess) {
    double diff_x_1, diff_y_1;  //edges of triangles
    double dist_1; //distance of guess from beacon
    diff_x_1 = guess.first - h_1; //bottom edge of triangle
    diff_y_1 = guess.second - k_1;
    dist_1 = sqrt(diff_x_1 * diff_x_1 + diff_y_1 * diff_y_1); //pytheor
    double circle1Deriv = diff_x_1/dist_1;
    if (dist_1 < r_1) {
        circle1Deriv *= -1;
    }
    return circle1Deriv;
}


double 
calcError(const vector<double>& readings, 
	const vector<pair<double,double>>& offsets, 
		const pair<double, double>  guess);

double 
calcPartialX(const vector<double>& readings, 
                const vector<pair<double,double>>& offsets,
			const pair<double, double>  guess);

double 
calcPartialY(const vector<double>& readings, 
                    	const vector<pair<double,double>>& offsets,
				const pair<double, double>  guess);
pair<double, double> 
RoverBeacon::steepest_descent(const vector<double>& readings, 
				const vector<pair<double,double>>& offsets,
                                  pair<double, double>  old_pos) {
    pair<double, double> new_pos;
    double x_deriv, y_deriv;
    double current_error, past_error;
    double error_diff = 10;
    double threshold = 0.00000005;
  
    new_pos = old_pos;
    current_error = numeric_limits<double>::max();
    past_error = numeric_limits<double>::min();
  
    while ( error_diff > threshold) {
        old_pos = new_pos;
        x_deriv = calcPartialX(readings, offsets, new_pos);
        y_deriv = calcPartialY(readings, offsets, new_pos);
        new_pos.first = new_pos.first - x_deriv * 0.1;
        new_pos.second = new_pos.second - y_deriv * 0.1;
        past_error = current_error;
        current_error = calcError(readings, offsets, new_pos);
        error_diff = abs(past_error - current_error);    
    }
    return new_pos;
}

double 
calcError(const vector<double>& readings, const vector<pair<double,double>>& offsets, const pair<double, double>  guess) {
    double error = 0;
    for (int i = 0; i < readings.size(); ++i) {
        error += beaconError(readings.at(i), 
				offsets.at(i).first, 
					offsets.at(i).second, guess);
    }
    return error;
}


double 
calcPartialX(const vector<double>& readings, 
		const vector<pair<double,double>>& offsets,
		const pair<double, double>  guess) {
    double partialX = 0;
    for (int i = 0; i < readings.size(); ++i) {
        partialX += beaconPartialX(readings.at(i), 
					offsets.at(i).first, 
						offsets.at(i).second,
								 guess);
    }
    return partialX;
}


double 
calcPartialY(const vector<double>& readings, 
		const vector<pair<double,double>>& offsets,
			const pair<double, double>  guess) {
    double partialY = 0;
    for (int i = 0; i < readings.size(); ++i) {
        partialY += beaconPartialY(readings.at(i), 
					offsets.at(i).first, 
						offsets.at(i).second,
							 guess);
    }
    return partialY;
}
