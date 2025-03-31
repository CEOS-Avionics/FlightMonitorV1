#ifndef _BMPMEASUREMENTMODEL_HPP_
#define _BMPMEASUREMENTMODEL_HPP_
 
#include "C:\Users\henri\OneDrive\Documentos\KalmanFilter\kalman-master\include\kalman\LinearizedMeasurementModel.hpp"
#include "systemModel.hpp"
   
template<typename T>
class bmpMeasurement : public Kalman::Vector<T, 1>
{
public:
    KALMAN_VECTOR(bmpMeasurement, T, 1)
    
    static constexpr size_t PZ = 0;

    T pz()  const { return (*this)[PZ]; }

    T& pz() { return (*this)[PZ]; }
};

template<typename T, template<class> class CovarianceBase = Kalman::StandardBase>
class bmpMeasurementModel : public Kalman::LinearizedMeasurementModel<State<T>, bmpMeasurement<T>, CovarianceBase>
{
public:
    
    // Atalhos para os vetores
    typedef State<T> S;
    typedef bmpMeasurement<T> M;

    bmpMeasurementModel()
    {
        // Setup jacobians. As these are static, we can define them once
        // and do not need to update them dynamically
        this->H.setZero();
        this->H( M::PZ, S::PZ ) = 1;
        
        this->V.setIdentity();
    }
    
    M h(const S& x) const
    {
        M measurement;
        
        // z = h(x) + v, com a definicao abaixo temos z = x + v
        measurement.pz() = x.pz();
        
        return measurement;
    }

};

#endif