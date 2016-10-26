//
// Created by doopy on 15/07/16.
//

#include <string.h>
#include <fstream>
#include <stdlib.h>
#include <ctype.h>
#include <list>

#include <gtsam/base/FastVector.h>

#include <gtsam/inference/Symbol.h>

#include <gtsam/geometry/Pose2.h>

#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/BearingRangeFactor.h>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearConjugateGradientOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>


using namespace std;
using namespace gtsam;
typedef boost::shared_ptr<Factor> sharedFactor;  ///< Shared pointer to a factor

class UnaryFactor: public NoiseModelFactor1<Pose2> {

    // The factor will hold a measurement consisting of an (X,Y) location
    // We could this with a Point2 but here we just use two doubles
    double mx_, my_;

public:
    /// shorthand for a smart pointer to a factor
    typedef boost::shared_ptr<UnaryFactor> shared_ptr;

    // The constructor requires the variable key, the (X, Y) measurement value, and the noise model
    UnaryFactor(Key j, double x, double y, const SharedNoiseModel& model):
            NoiseModelFactor1<Pose2>(model, j), mx_(x), my_(y) {}

    virtual ~UnaryFactor() {}

    // Using the NoiseModelFactor1 base class there are two functions that must be overridden.
    // The first is the 'evaluateError' function. This function implements the desired measurement
    // function, returning a vector of errors when evaluated at the provided variable value. It
    // must also calculate the Jacobians for this measurement function, if requested.
    Vector evaluateError(const Pose2& q, boost::optional<Matrix&> H = boost::none) const
    {
        // The measurement function for a GPS-like measurement is simple:
        // error_x = pose.x - measurement.x
        // error_y = pose.y - measurement.y
        // Consequently, the Jacobians are:
        // [ derror_x/dx  derror_x/dy  derror_x/dtheta ] = [1 0 0]
        // [ derror_y/dx  derror_y/dy  derror_y/dtheta ] = [0 1 0]
        if (H) (*H) = (Matrix(2,3) << 1.0,0.0,0.0, 0.0,1.0,0.0);
        return (Vector(2) << q.x() - mx_, q.y() - my_);
    }

    // The second is a 'clone' function that allows the factor to be copied. Under most
    // circumstances, the following code that employs the default copy constructor should
    // work fine.
    virtual gtsam::NonlinearFactor::shared_ptr clone() const {
        return boost::static_pointer_cast<gtsam::NonlinearFactor>(
                gtsam::NonlinearFactor::shared_ptr(new UnaryFactor(*this))); }

    // Additionally, we encourage you the use of unit testing your custom factors,
    // (as all GTSAM factors are), in which you would need an equals and print, to satisfy the
    // GTSAM_CONCEPT_TESTABLE_INST(T) defined in Testable.h, but these are not needed below.

}; // UnaryFactor

int main (int argc, char **argv) {

    cout << "EXECUTING gtsamSim\n" << endl;
    ifstream fs("/home/doopy/Documents/gtsamSimulationData/datasets/noisyTestData.txt");
    ifstream noiseFile("/home/doopy/Documents/gtsamSimulationData/datasets/uncertainties.txt");

    ofstream resultsFile("/home/doopy/Documents/gtsamSimulationData/results/results.txt");

    string bamn, bamn2, landmark;

    float noise[10];

    float x, y, thet, E, N, dx, dy, dthet, range;
    int mark,id;
    int count = 1;

    while(getline(noiseFile,bamn2))
    {
        char * noiseElements;

        noiseElements = strtok(&bamn2[0]," ");

        int i = 0;

        while(noiseElements != NULL)
        {

            noise[i] = strtof(noiseElements,NULL);
            noiseElements = strtok(NULL, " ");
            ++i;
        }
    }
    //Create a new non-linear factor graph
    NonlinearFactorGraph graph;

    //Create a variable to hold each epoch's initial pose estimate
    Values initial;

    //Create a prior pose estimate and the associated covariance matrix to initialize the algorithm
    //Add the prior mean and covariance matrix to the factor graph as a new Pose2 typed factor
    Pose2 priorMean(0.0,0.0,0.0);
    noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas((Vector(3) << noise[0], noise[1], noise[2]));
    graph.add(PriorFactor<Pose2>(Symbol('x',0),priorMean, priorNoise));

    initial.insert(Symbol('x',0), Pose2(0.0,0.0,0.0));
    while(getline(fs,bamn))
    {
        if (!isalpha(bamn[0]))
        {
            mark = bamn.find(" ");
            id = atoi(bamn.substr(0,mark).c_str());

            x = strtof(bamn.substr(mark+1,bamn.find(" ",mark+1) - mark).c_str(),NULL);
            mark = bamn.find(" ",mark+1);

            y = strtof(bamn.substr(mark+1,bamn.find(" ",mark+1) - mark).c_str(),NULL);
            mark = bamn.find(" ",mark+1);

            thet = strtof(bamn.substr(mark+1,bamn.find(" ",mark+1) - mark).c_str(),NULL);
            mark = bamn.find(" ",mark+1);

            //Add the initial pose estimate into the container for initial estimates
            initial.insert(Symbol('x',id),Pose2(x,y,thet));

            E = strtof(bamn.substr(mark+1,bamn.find(" ",mark+1) - mark).c_str(),NULL);
            mark = bamn.find(" ",mark+1);

            N = strtof(bamn.substr(mark+1,bamn.find(" ",mark+1) - mark).c_str(),NULL);
            mark = bamn.find(" ",mark+1);

            //Create a covariance matrix for the localization measurement
            noiseModel::Diagonal::shared_ptr unaryNoise = noiseModel::Diagonal::Sigmas((Vector(2) << noise[3], noise[4]));
            //Add the localization measurement and its covariance matrix to the non-linear factor graph
            graph.add(boost::make_shared<UnaryFactor>(Symbol('x',id), E, N, unaryNoise));

            dx = strtof(bamn.substr(mark+1,bamn.find(" ",mark+1) - mark).c_str(),NULL);
            mark = bamn.find(" ",mark+1);

            dy = strtof(bamn.substr(mark+1,bamn.find(" ",mark+1) - mark).c_str(),NULL);
            mark = bamn.find(" ",mark+1);

            dthet = strtof(bamn.substr(mark+1,bamn.length() - mark).c_str(),NULL);

            if(!fs.eof()) {

                //Create an odometry measurement and its associated noise model and add it to the factor graph
                Pose2 odometry(dx, dy, dthet);
                noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Sigmas((Vector(3) << noise[5],noise[6],noise[7]));

                graph.add(BetweenFactor<Pose2>(Symbol('x',id-1), Symbol('x',id), odometry, odometryNoise));
            }

            count = 1;
        }
        else
        {
            mark = bamn.find(" ");
            landmark = bamn.substr(0,mark).c_str();

            x = strtof(bamn.substr(mark+1,bamn.find(" ",mark+1) - mark).c_str(),NULL);
            mark = bamn.find(" ",mark+1);

            y = strtof(bamn.substr(mark+1,bamn.find(" ",mark+1) - mark).c_str(),NULL);
            mark = bamn.find(" ",mark+1);

            if(landmark[0] == 'l')
            {
                initial.insert(Symbol('l',count),Point2(x,y));
            }

            range = sqrt(pow(x,2) + pow(y,2));
            Rot2 bear(atan2(y,x));
            noiseModel::Diagonal::shared_ptr measurementNoise = noiseModel::Diagonal::Sigmas((Vector(2) << noise[8], noise[9])); // 0.1 rad std on bearing, 20cm on range

            graph.add(BearingRangeFactor<Pose2, Point2>(Symbol('x',id), Symbol('l',count), bear, range, measurementNoise));

            ++count;
        }

    }

    const Values resultLM = LevenbergMarquardtOptimizer(graph,initial).optimize();
    Marginals marginalsLM(graph, resultLM);

    const Values resultGN = GaussNewtonOptimizer(graph,initial).optimize();
    Marginals marginalsGN(graph, resultLM);

    const Values resultNL = NonlinearConjugateGradientOptimizer(graph, initial).optimize();
    Marginals marginalsNL(graph, resultLM);

    resultsFile << "Levenberg Marquardt Final Result:" << endl;

    for(Values::const_iterator key_value = resultLM.begin(); key_value != resultLM.end(); ++key_value)
    {
        if (DefaultKeyFormatter(key_value->key)[0] == 'x')
        {

            const Pose2* p = dynamic_cast<const Pose2*>(&key_value->value);
            resultsFile << DefaultKeyFormatter(key_value->key) << " " << p->t() << " " << p->r().theta() << endl;
        }
        else if (DefaultKeyFormatter(key_value->key)[0] == 'l')
        {

            const Point2* p = dynamic_cast<const Point2*>(&key_value->value);
            resultsFile << DefaultKeyFormatter(key_value->key) << " " << *p << endl;

        }
    }
    resultsFile << "- - - - - - - -" << endl;
    resultsFile << "Gauss Newton Final Result:" << endl;

    for(Values::const_iterator key_value = resultGN.begin(); key_value != resultGN.end(); ++key_value)
    {
        if (DefaultKeyFormatter(key_value->key)[0] == 'x')
        {

            const Pose2* p = dynamic_cast<const Pose2*>(&key_value->value);
            resultsFile << DefaultKeyFormatter(key_value->key) << " " << p->t() << " " << p->r().theta() << endl;
        }
        else if (DefaultKeyFormatter(key_value->key)[0] == 'l')
        {

            const Point2* p = dynamic_cast<const Point2*>(&key_value->value);
            resultsFile << DefaultKeyFormatter(key_value->key) << " " << *p << endl;

        }
    }
    resultsFile << "- - - - - - - -" << endl;
    resultsFile << "Nonlinear Final Result:" << endl;

    for(Values::const_iterator key_value = resultNL.begin(); key_value != resultNL.end(); ++key_value)
    {
        if (DefaultKeyFormatter(key_value->key)[0] == 'x')
        {

            const Pose2* p = dynamic_cast<const Pose2*>(&key_value->value);
            resultsFile << DefaultKeyFormatter(key_value->key) << " " << p->t() << " " << p->r().theta() << endl;
        }
        else if (DefaultKeyFormatter(key_value->key)[0] == 'l')
        {

            const Point2* p = dynamic_cast<const Point2*>(&key_value->value);
            resultsFile << DefaultKeyFormatter(key_value->key) << " " << *p << endl;

        }
    }

    resultsFile << "end end end" << endl;

    noiseFile.clear();
    noiseFile.close();

    resultsFile.clear();
    resultsFile.close();

    fs.clear();
    fs.close();

}
