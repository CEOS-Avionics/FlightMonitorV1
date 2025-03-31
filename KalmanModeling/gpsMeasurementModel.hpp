#ifndef _GPSMEASUREMENTMODEL_HPP_
#define _GPSMEASUREMENTMODEL_HPP_

#include "C:\Users\henri\OneDrive\Documentos\KalmanFilter\kalman-master\include\kalman\LinearizedMeasurementModel.hpp"
#include "systemModel.hpp"
#include <cmath>

/**
 * @brief Measurement vector measuring the robot position
 *
 * @param T Numeric scalar type
 */
template<typename T>
class gpsMeasurement : public Kalman::Vector<T, 5>
{
public:
    KALMAN_VECTOR(gpsMeasurement, T, 5)
    
    static constexpr size_t PX = 0;
    static constexpr size_t PY = 1;
    static constexpr size_t PZ = 2;
    static constexpr size_t V = 3;
    static constexpr size_t THETA = 4;
    
    T px() const { return (*this)[ PX ]; }
    T py() const { return (*this)[ PY ]; }
    T pz() const { return (*this)[ PZ ]; }
    T v() const { return (*this)[ V ]; }
    T theta() const { return (*this)[ THETA ]; }
    
    T& px() { return (*this)[ PX ]; }
    T& py() { return (*this)[ PY ]; }
    T& pz() { return (*this)[ PZ ]; }
    T& v() { return (*this)[ V ]; }
    T& theta() { return (*this)[ THETA ]; }
};

/**
 * @brief Measurement model for measuring the position of the robot
 *        using two beacon-landmarks
 *
 * This is the measurement model for measuring the position of the robot.
 * The measurement is given by two landmarks in the space, whose positions are known.
 * The robot can measure the direct distance to both the landmarks, for instance
 * through visual localization techniques.
 *
 * @param T Numeric scalar type
 * @param CovarianceBase Class template to determine the covariance representation
 *                       (as covariance matrix (StandardBase) or as lower-triangular
 *                       coveriace square root (SquareRootBase))
 */
template<typename T, template<class> class CovarianceBase = Kalman::StandardBase>
class gpsMeasurementModel : public Kalman::LinearizedMeasurementModel<State<T>, gpsMeasurement<T>, CovarianceBase>
{
public:
    //! State type shortcut definition
    typedef  State<T> S;
    
    //! Measurement type shortcut definition
    typedef  gpsMeasurement<T> M;
    
    gpsMeasurementModel()
    {
        // Tanto aqui (no gps) quanto no bmp, as jacobianas de medida sao estaticas.
        this->H.setZero();
        this->H( M::PX, S::PX) = 1;
        this->H( M::PY, S::PY) = 1;
        this->H( M::PZ, S::PZ) = 1;

        this->V.setIdentity();
    }
    
    /**
     * @brief Definition of (possibly non-linear) measurement function
     *
     * This function maps the system state to the measurement that is expected
     * to be received from the sensor assuming the system is currently in the
     * estimated state.
     *
     * @param [in] x The system state in current time-step
     * @returns The (predicted) sensor measurement for the system state
     */
    M h(const S& x) const
    {
        M measurement;
        
        // O gps ja nos fornece as medidas de posicao e velocidade no referencial NED
        measurement.px() = x.px();
        measurement.py() = x.py();
        measurement.pz() = x.pz();
        measurement.v() = sqrt(x.vx() * x.vx() + x.vy() * x.vy());
        measurement.theta() = (atan2(x.vx(), x.vy())/M_PI)*180; // Em graus contado no sentido horario a partir do norte.
        
        return measurement;
    }

protected:

    void updateJacobians(const S& x)
    {   
        if(x.vx() != 0 || x.vy() != 0) // garante que não haja divisão por 0.
        { 
            this->H(M::V, S::VX) = x.vx() / sqrt(x.vx() * x.vx() + x.vy() * x.vy());
            this->H(M::V, S::VY) = x.vy() / sqrt(x.vx() * x.vx() + x.vy() * x.vy());
            this->H(M::THETA, S::VX) = x.vy() / (x.vx() * x.vx() + x.vy() * x.vy());
            this->H(M::THETA, S::VY) = -x.vx() / (x.vx() * x.vx() + x.vy() * x.vy());
        }
    }
};

#endif