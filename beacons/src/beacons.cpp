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

bool isLOS(double amp1, double amp2, double amp3, double pre1, double pre2, double imp_pwr){
    double fpp, rxp;

    fpp = pow(amp1, 2) + pow(amp2, 2) + pow(amp3, 2);
    fpp /= pow( pre1, 2);
    fpp = 10 * log10(fpp);
    //cout << fpp << "\n";
    rxp =  double(1 << 17) * imp_pwr;
    rxp /= pow(pre2 ,2);
    rxp = 10 * log10(rxp);
    //cout << rxp << "\n";
    //cout << "Difference is: " << fpp - rxp << "\n"; 
    return true;
}

ostream& operator<<(ostream& out, const BeaconEnv& b){
    out << b.position.first << "    " << b.position.second << "\n";
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
    unsigned resp = input[1];
    isLOS(input[10], input[11], input[12], input[16], input[24], input[21]);
    //cout << input[2] << " dist  " << input[8] << "\n";
    b.RoverBeacons[resp].updateReading(input[2], input[8]);
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

double calcDist(pair<double, double> p1, pair<double, double> p2 );

void BeaconEnv::updatePosition(){
    auto it = RoverBeacons.begin();
    auto endIt = RoverBeacons.end();
    for(; it != endIt;++it){
        it->second.updatePosition();
    }
}



void
RoverBeacon::updatePosition(){
    vector<double> readings;
    vector<pair<double,double>> offsets;
    auto it = beaconReadings.begin();
    auto endIt = beaconReadings.end();

#ifdef DEBUG
    cout << "Calculating the position of rover beacon " << id <<  " with an old position of"
        << position.first << "," << position.second << " and the following readings\n";
#endif
    for (; it != endIt; ++it) {
#ifdef DEBUG
        cout << "        " << "Sieve Beacon " << it->first << ": " << it->second.reading << "\n";
#endif
            if(it-> second.reading){
                readings.push_back(it->second.reading);
                offsets.push_back(it->second.offset);
            }
    }
    
    if(readings.size() > 1){
        position = steepest_descent(readings,offsets,position);
    }

#ifdef DEBUG
    cout << "The position of is " << position.first << "," << position.second << "\n";
#endif

}

pair<double, double> 
RoverBeacon::steepest_descent(const vector<double>& readings, 
        const vector<pair<double,double>>& offsets,
        pair<double, double>  old_pos) {
    const double degree = M_PI / 180;
    pair<double, double> new_pos;
    pair<double, double> pos;
    //double x_deriv, y_deriv;
    //double current_error;
    //double threshold = 0.000000001;
    double radius;
    double dist=0, sum=0,min_sum = numeric_limits<double>::max();
    new_pos = old_pos;
    //current_error = numeric_limits<double>::max();


    /*
    while ( current_error > threshold) {
        old_pos = new_pos;
        x_deriv = calcPartialX(readings, offsets, new_pos);
        //cout << "x deriv = " << x_deriv;
        y_deriv = calcPartialY(readings, offsets, new_pos);
        //cout << "    y deriv = " << y_deriv << endl;
        new_pos.first = new_pos.first + x_deriv * 0.1;
        new_pos.second = new_pos.second + y_deriv * 0.1;
        current_error = calcDist(old_pos, new_pos);
        //cout << "Error Diff: " << current_error << "\n";
        //cout << "x: " << new_pos.first<< ", y: " << new_pos.second << "\n";
    }
    */
    vector< pair<double, double> > points;
    for( unsigned i =0; i < readings.size();i++){
        radius = readings[i] ;
        sum=0;
        dist=0;
        min_sum=numeric_limits<double>::max();
        new_pos=pair<double, double>(0,0);
        for( double angle = 0; angle < M_PI; angle+= (degree/8.0)){
            pos.first = cos(angle) * radius + offsets[i].first ;
            pos.second = sin(angle) * radius;
            for(unsigned j = 0; j < offsets.size(); j++){
                if(j == i)
                    continue;
                dist = calcDist( pos, offsets[j] ); 
                sum += fabs(dist - readings[j]);
            }
            if(sum < min_sum){
                min_sum = sum;
                new_pos = pos;

            }
            dist = 0;
            sum = 0;
        }
       points.push_back(new_pos); 
    }

    new_pos=pair<double, double>(0,0);

    for(auto point : points){
        new_pos.first += point.first;
        new_pos.second += point.second;
    }

    new_pos.first /= points.size();
    new_pos.second /= points.size();

    cout << radius << "\n";
    
    cout << "x: " << new_pos.first<< ", y: " << new_pos.second << "\n";
    
    return new_pos;
}

double calcDist(pair<double, double> p1, pair<double, double> p2 ){
    double x = p1.first - p2.first;
    double y = p1.second - p2.second;
    return sqrt( (x*x) + (y*y)  );
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
        double add = beaconPartialX(readings.at(i), 
                offsets.at(i).first, 
                offsets.at(i).second,
                guess);
        //cout << "Partial X, adding circle: " << i << " with x value:  " << add << "\n";
        partialX += add;
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
    double sin_circ = diff_y_1/dist_1;
    double y_circ = sin_circ * r_1;
    if( y_circ == 0 || diff_y_1 == 0){
        return 0;
    }
    double ratio = -(diff_y_1 / y_circ);
    if (dist_1 < r_1) {
        ratio = 1/ratio;
        ratio *= -1;
        ratio -= 1;
    }else{
        ratio += 1;
    }
    return ratio;
}

double 
beaconPartialX(double r_1, double h_1, double k_1, 
        const pair<double, double>& guess) {
    double diff_x_1, diff_y_1;  //edges of triangles
    double dist_1; //distance of guess from beacon
    diff_x_1 = guess.first - h_1; //bottom edge of triangle
    diff_y_1 = guess.second - k_1;
    dist_1 = sqrt(diff_x_1 * diff_x_1 + diff_y_1 * diff_y_1); //pytheor
    double cos_circ = diff_x_1/dist_1;
    double x_circ = cos_circ * r_1;
    if(x_circ == 0 || cos_circ == 0){
        return 0;
    }   
    double ratio = -(diff_x_1 / x_circ);
    if (dist_1 < r_1) {
        ratio = 1/ratio;
        ratio *= -1;
        ratio -= 1;
    }else{
        ratio += 1;
    }
    return ratio;

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


