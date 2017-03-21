#include "beacons.h"

#define DEBUG
#define FULLDEBUG
using std::numeric_limits;
using std::cout; 
using std::endl;
using std::pair; 
using std::vector;
using std::unordered_map; 
using std::string;
using std::ostream;using std::istream;
using std::stringstream;

double 
calcoutor(const vector<double>& readings, 
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

double beaconError(double r_1, double h_1, double k_1, 
        const pair<double, double>& guess);

double beaconPartialX(double r_1, double h_1, double k_1, 
        const pair<double, double>& guess); 

double beaconPartialY(double r_1, double h_1, double k_1, 
        const pair<double, double>& guess);

ostream& operator<<(ostream& out, const BeaconEnv& b){
    out << b.position.first << "    " << b.position.second << "\n";
    out << "at an orienation of " <<  b.orientation;
    return out << "\n";
}

istream& operator>>(istream& in, BeaconEnv& b){
    string token, sentence;
    stringstream iss;
    vector<double> input;

    getline(in, sentence);
    iss << sentence;
    while( getline(iss, token, ',') ){
        input.push_back( atof(token.c_str()) );
    }
    // responder is the 3rd data value which is not the one connected to a computer
    unsigned resp = input[2];
    b.RoverBeacons[resp].updateReading(input[1], input[8]);
    b.updateOrientation();
    b.positionUpdated = false;
    b.getPosition();

    return in;
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
}

double 
RoverBeacon::getBias(int _id){
    beaconIterator = beaconReadings.find(_id);
    return beaconIterator->second.bias;
}

void 
RoverBeacon::updateBias(int _id, double _bias){
    beaconIterator = beaconReadings.find(_id);
    beaconIterator->second.bias = _bias;
}

pair<double,double>
BeaconEnv::getPosition(){
    //Position of the Beacon should only be updated 
    //if new readings have been taken
    if(!positionUpdated) {
        updatePosition();
        positionUpdated = true;
    }
    return position;
}

void BeaconEnv::updateOrientation(){
    float dist = 0.64;
    unsigned middle = 50;
    float front_reading = RoverBeacons[64].beaconReadings[middle].reading;
    float back_reading = RoverBeacons[5].beaconReadings[middle].reading;
    float cos_val = pow(back_reading, 2) + pow(dist, 2) - pow(front_reading, 2);
    cos_val /= ((-2)*back_reading * front_reading);
    orientation =  M_PI - acos(cos_val);
}

float BeaconEnv::getOrientation(){
    return orientation;
}

void
BeaconEnv::updatePosition(){
    vector<double> readings;
    vector<pair<double,double>> offsets;
    auto it = RoverBeacons.begin();
    auto endIt = RoverBeacons.end();

#ifdef DEBUG
    cout << "Calculating the position of rover beacon with an old position of"
        << position.first << "," << position.second << " and the following readings\n";
#endif
    for (; it != endIt; ++it) {
#ifdef DEBUG
        cout << "        " << "Rover Beacon " << it->first << ": " << it->second.reading << "\n";
#endif
        auto inner_it = it->second.beaconReadings.begin();
        auto endinner_it = it->second.beaconReadings.end();
            
        for (; inner_it != endinner_it; ++inner_it) {
#ifdef DEBUG
            cout << "        " << "Sieve Beacon " << inner_it->first << ": " << inner_it->second.reading << "\n";
#endif        
            readings.push_back(inner_it->second.reading);
            offsets.push_back(inner_it->second.offset);
        }
    }

    position = steepest_descent(readings,offsets, position);
#ifdef DEBUG
    cout << "The position of is " << position.first << "," << position.second << "\n";
#endif

}

pair<double, double> 
BeaconEnv::steepest_descent(const vector<double>& readings, 
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
        cout << "x deriv = " << x_deriv;
        y_deriv = calcPartialY(readings, offsets, new_pos);
        cout << "    y deriv = " << y_deriv << endl;
        new_pos.first = new_pos.first - x_deriv * 0.1;
        new_pos.second = new_pos.second - y_deriv * 0.1;
        past_error = current_error;
        current_error = calcoutor(readings, offsets, new_pos);
        error_diff = abs(past_error - current_error);    
    }
    return new_pos;
}

double 
calcoutor(const vector<double>& readings, const vector<pair<double,double>>& offsets, const pair<double, double>  guess) {
    double error = 0;
    for (unsigned i = 0; i < readings.size(); ++i) {
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
    for (unsigned i = 0; i < readings.size(); ++i) {
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
    for (unsigned i = 0; i < readings.size(); ++i) {
        partialY += beaconPartialY(readings.at(i), 
                offsets.at(i).first, 
                offsets.at(i).second,
                guess);
    }
    return partialY;
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


