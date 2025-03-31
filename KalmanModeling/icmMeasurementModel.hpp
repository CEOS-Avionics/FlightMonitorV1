#ifndef _ICMMEASUREMENTMODEL_HPP_
#define _ICMMEASUREMENTMODEL_HPP_

#include "C:\Users\henri\OneDrive\Documentos\KalmanFilter\kalman-master\include\kalman\LinearizedMeasurementModel.hpp"
#include "systemModel.hpp"

template<typename T>
class icmMeasurement : public Kalman::Vector<T, 9>
{
public:
    KALMAN_VECTOR(icmMeasurement, T, 9)

    //! Accelerometer readings
    static constexpr size_t AX = 0;
    static constexpr size_t AY = 1;
    static constexpr size_t AZ = 2;
    //! Gyroscope readings
    static constexpr size_t WX = 3;
    static constexpr size_t WY = 4;
    static constexpr size_t WZ = 5;
    //! Magnetometer readings
    static constexpr size_t MX = 6;
    static constexpr size_t MY = 7;
    static constexpr size_t MZ = 8;
    
    T ax() const { return (*this)[ AX ]; }
    T ay() const { return (*this)[ AY ]; }
    T az() const { return (*this)[ AZ ]; }
    T wx() const { return (*this)[ WX ]; }
    T wy() const { return (*this)[ WY ]; }
    T wz() const { return (*this)[ WZ ]; }
    T mx() const { return (*this)[ MX ]; }
    T my() const { return (*this)[ MY ]; }
    T mz() const { return (*this)[ MZ ]; }
    
    T& ax() { return (*this)[ AX ]; }
    T& ay() { return (*this)[ AY ]; }
    T& az() { return (*this)[ AZ ]; }
    T& wx() { return (*this)[ WX ]; }
    T& wy() { return (*this)[ WY ]; }
    T& wz() { return (*this)[ WZ ]; }
    T& mx() { return (*this)[ MX ]; }
    T& my() { return (*this)[ MY ]; }
    T& mz() { return (*this)[ MZ ]; }
};


/**
 * @brief Measurement model for imu + magnetometer
 *
 * @param T Numeric scalar type
 * @param CovarianceBase Class template to determine the covariance representation
 *                       (as covariance matrix (StandardBase) or as lower-triangular
 *                       coveriace square root (SquareRootBase))
 */
template<typename T, template<class> class CovarianceBase = Kalman::StandardBase>
class icmMeasurementModel : public Kalman::LinearizedMeasurementModel<State<T>, icmMeasurement<T>, CovarianceBase>
{
public:

    typedef State<T> S;
    typedef icmMeasurement<T> M;
    
    icmMeasurementModel()
    {
        // Estou com preguiça de fazer a modelagem do ruído 
        // (inclusive eu não sei como fazer isso direito)
        // então vou deixar a matriz de covariância como identidade
        // V = dh/dv
        this->V.setIdentity();

        // H = dh/dx (Jacobiana da função de medição em relação ao estado)
        this->H.setZero();
        // Agora a gente vai preencher os elementos da jacobiana H que não precisam ser atualizados
        this->H(M::WX, S::WX) = 1;
        this->H(M::WY, S::WY) = 1;
        this->H(M::WZ, S::WZ) = 1;
        this->H(M::AX, S::ABX) = 1;
        this->H(M::AY, S::ABY) = 1;
        this->H(M::AZ, S::ABZ) = 1;
        this->H(M::MX, S::MBX) = 1;
        this->H(M::MY, S::MBY) = 1;
        this->H(M::MZ, S::MBZ) = 1;
        this->H(M::WX, S::GBX) = 1;
        this->H(M::WY, S::GBY) = 1;
        this->H(M::WZ, S::GBZ) = 1;
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
        
        T q0 = x.q0();
        T q1 = x.q1(); 
        T q2 = x.q2();
        T q3 = x.q3();
        T ax = x.ax();
        T ay = x.ay();
        T az = x.az() - 9.81; // Considerar que a gravidade também é medida pelo acelerômetro.
        T mx = x.mx();
        T my = x.my();
        T mz = x.mz();
        T R_inv[3][3] = {
            {1 - 2 * (q2 * q2 + q3 * q3), 2 * (q1 * q2 + q0 * q3), 2 * (q1 * q3 - q0 * q2)},
            {2 * (q1 * q2 - q0 * q3), 1 - 2 * (q1 * q1 + q3 * q3), 2 * (q2 * q3 + q0 * q1)},
            {2 * (q1 * q3 + q0 * q2), 2 * (q2 * q3 - q0 * q1), 1 - 2 * (q1 * q1 + q2 * q2)}
        };
        
        // A velocidade angular medida deve ser a mesma que está no vetor de estados mais o bias
        measurement.wx() = x.wx() + x.gbx();
        measurement.wy() = x.wy() + x.gby();
        measurement.wz() = x.wz() + x.gbz();
        // A aceleração medida deve ser a rotação inversa da que está no vetor de estados, mais o bias
        measurement.ax() = R_inv[0][0] * ax + R_inv[0][1] * ay + R_inv[0][2] * az + x.abx();
        measurement.ay() = R_inv[1][0] * ax + R_inv[1][1] * ay + R_inv[1][2] * az + x.aby();
        measurement.az() = R_inv[2][0] * ax + R_inv[2][1] * ay + R_inv[2][2] * az + x.abz();
        // O campo magnético medido deve ser a rotação inversa da que está no vetor de estados, mais o bias
        measurement.mx() = R_inv[0][0] * mx + R_inv[0][1] * my + R_inv[0][2] * mz + x.mbx();
        measurement.my() = R_inv[1][0] * mx + R_inv[1][1] * my + R_inv[1][2] * mz + x.mby();
        measurement.mz() = R_inv[2][0] * mx + R_inv[2][1] * my + R_inv[2][2] * mz + x.mbz();
        
        return measurement;
    }

protected:

    /**
     * @brief Update jacobian matrices for the system state transition function using current state
     *
     * This will re-compute the (state-dependent) elements of the jacobian matrices
     * to linearize the non-linear measurement function \f$h(x)\f$ around the
     * current state \f$x\f$.
     *
     * @note This is only needed when implementing a LinearizedSystemModel,
     *       for usage with an ExtendedKalmanFilter or SquareRootExtendedKalmanFilter.
     *       When using a fully non-linear filter such as the UnscentedKalmanFilter
     *       or its square-root form then this is not needed.
     *
     * @param x The current system state around which to linearize
     * @param u The current system control input
     */
    void updateJacobians(const S& x)
    {
        T q0 = x.q0();
        T q1 = x.q1(); 
        T q2 = x.q2();
        T q3 = x.q3();
        T ax = x.ax();
        T ay = x.ay();
        T az = x.az() - 9.81; // Considerar que a gravidade também é medida pelo acelerômetro.
        T mx = x.mx();
        T my = x.my();
        T mz = x.mz();
        T R_inv[3][3] = {
            {1 - 2 * (q2 * q2 + q3 * q3), 2 * (q1 * q2 + q0 * q3), 2 * (q1 * q3 - q0 * q2)},
            {2 * (q1 * q2 - q0 * q3), 1 - 2 * (q1 * q1 + q3 * q3), 2 * (q2 * q3 + q0 * q1)},
            {2 * (q1 * q3 + q0 * q2), 2 * (q2 * q3 - q0 * q1), 1 - 2 * (q1 * q1 + q2 * q2)}
        };

        // Linha 0: derivadas de ax
        this->H(M::AX, S::Q0) = 2 * q3 * ay - 2 * q2 * az;
        this->H(M::AX, S::Q1) = 2 * q2 * ay + 2 * q3 * az;
        this->H(M::AX, S::Q2) = -4 * q2 * ax + 2 * q1 * ay - 2 * q0 * az;
        this->H(M::AX, S::Q3) = -4 * q3 * ax + 2 * q0 * ay + 2 * q1 * az;
        // Linha 1: derivadas de ay
        this->H(M::AY, S::Q0) = -2 * q3 * ax + 2 * q1 * az;
        this->H(M::AY, S::Q1) = 2 * q2 * ax - 4 * q1 * ay + 2 * q0 * az;
        this->H(M::AY, S::Q2) = 2 * q1 * ax + 2 * q3 * az;
        this->H(M::AY, S::Q3) = -2 * q0 * ax - 4 * q3 * ay + 2 * q2 * az;
        // Linha 2: derivadas de az
        this->H(M::AZ, S::Q0) = 2 * q2 * ax - 2 * q1 * ay;
        this->H(M::AZ, S::Q1) = 2 * q3 * ax - 2 * q0 * ay - 4 * q1 * az;
        this->H(M::AZ, S::Q2) = 2 * q0 * ax + 2 * q3 * ay - 4 * q2 * az;
        this->H(M::AZ, S::Q3) = 2 * q1 * ax + 2 * q2 * ay;
        // Linha 6: derivadas de mx
        this->H(M::MX, S::Q0) = 2 * q3 * my - 2 * q2 * mz;
        this->H(M::MX, S::Q1) = 2 * q2 * my + 2 * q3 * mz;
        this->H(M::MX, S::Q2) = -4 * q2 * mx + 2 * q1 * my - 2 * q0 * mz;
        this->H(M::MX, S::Q3) = -4 * q3 * mx + 2 * q0 * my + 2 * q1 * mz;
        // Linha 7: derivadas de my
        this->H(M::MY, S::Q0) = -2 * q3 * mx + 2 * q1 * mz;
        this->H(M::MY, S::Q1) = 2 * q2 * mx - 4 * q1 * my + 2 * q0 * mz;
        this->H(M::MY, S::Q2) = 2 * q1 * mx + 2 * q3 * mz;
        this->H(M::MY, S::Q3) = -2 * q0 * mx - 4 * q3 * my + 2 * q2 * mz;
        // Linha 8: derivadas de mz
        this->H(M::MZ, S::Q0) = 2 * q2 * mx - 2 * q1 * my;
        this->H(M::MZ, S::Q1) = 2 * q3 * mx - 2 * q0 * my - 4 * q1 * mz;
        this->H(M::MZ, S::Q2) = 2 * q0 * mx + 2 * q3 * my - 4 * q2 * mz;
        this->H(M::MZ, S::Q3) = 2 * q1 * mx + 2 * q2 * my;

        // Agora deve-se inserir as proprias matrizes de rotacao inversas ao derivar 
        // a aceleracao em relacao a ela mesma e o campo magnetico em relacao a ele mesmo
        // a matriz de rotacao reversa funciona como uma constante

        // Preenchendo a Jacobiana H para a aceleração
        this->H(M::AX, S::AX) = R_inv[0][0];
        this->H(M::AX, S::AY) = R_inv[0][1];
        this->H(M::AX, S::AZ) = R_inv[0][2];
        this->H(M::AY, S::AX) = R_inv[1][0];
        this->H(M::AY, S::AY) = R_inv[1][1];
        this->H(M::AY, S::AZ) = R_inv[1][2];
        this->H(M::AZ, S::AX) = R_inv[2][0];
        this->H(M::AZ, S::AY) = R_inv[2][1];
        this->H(M::AZ, S::AZ) = R_inv[2][2];

        // Preenchendo a Jacobiana H para o campo magnético
        this->H(M::MX, S::MX) = R_inv[0][0];
        this->H(M::MX, S::MY) = R_inv[0][1];
        this->H(M::MX, S::MZ) = R_inv[0][2];
        this->H(M::MY, S::MX) = R_inv[1][0];
        this->H(M::MY, S::MY) = R_inv[1][1];
        this->H(M::MY, S::MZ) = R_inv[1][2];
        this->H(M::MZ, S::MX) = R_inv[2][0];
        this->H(M::MZ, S::MY) = R_inv[2][1];
        this->H(M::MZ, S::MZ) = R_inv[2][2]; 
    }
};

#endif