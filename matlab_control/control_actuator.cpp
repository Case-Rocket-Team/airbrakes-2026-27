#include <fstream>
#include <sstream>
#include <iostream>
#include <string>
#include <vector> 
#include <array>
#include <libInterpolate/Interpolate.hpp>


int main() {
    std::ifstream file("CDA_data.csv");

    if (!file.is_open()) {
        std::cout << "Unable to open file";
        return 1;
    }
    
    std::vector<std::array<double, 3>> A;
    std::string line;

    while (std::getline(file, line)) {
        char comma1, comma2;
        double x, v, cda;

        std::stringstream ss(line);

        if (ss >> x >> comma1 >> v >> comma2 >> cda) {

            std::array<double, 3> values = {x, v, cda};

            A.push_back(values);
        }
    }

    if (A.empty()){
        std::cout << "Data read was unsuccesful or file is empty";
        return 1;
    }

    std::vector<double> xs;
    std::vector<double> vs;
    std::vector<double> cdas;

    for (const auto& row : A) {

        xs.push_back(row[0]);
        vs.push_back(-row[1]);
        cdas.push_back(row[2]);
    }

    _2D::LinearDelaunayTriangleInterpolator<double> interp;
    interp.setData(xs, vs, cdas);
    
    std::cout << "Number of data points: " << A.size() << "\n";
    std::cout << "First CDA value: " << cdas[0] << "\n";

    double result = interp(650.4557, 233.6384);
    std::cout << "Interpolated CDA: " << result << "\n";
}