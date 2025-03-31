#ifndef KALMAN_EXAMPLES1_ROBOT_SYSTEMMODEL_HPP_
#define KALMAN_EXAMPLES1_ROBOT_SYSTEMMODEL_HPP_

#include "C:\Users\henri\OneDrive\Documentos\KalmanFilter\kalman-master\include\kalman\LinearizedSystemModel.hpp"

/**
 * @brief System state vector-type for a 3DOF planar robot
 *
 * This is a system state for a very simple planar robot that
 * is characterized by its (x,y)-Position and angular orientation.
 *
 * @param T Numeric scalar type
 */
template<typename T>
class State : public Kalman::Vector<T, 28>
{
public:
    KALMAN_VECTOR(State, T, 28)

    // ATENCAO!!! Por preguica eu deixei o nome dos indices da aceleracao e do campo magnetico
    // tambem como se fossem eles medidos nos eixos xyz, tais como a aceleracao e o campo no icm
    // mas aqui no vetor de estados e importante lembrar que a aceleracao e o campo sao medidas 
    // no referencial NED! O mesmo vale para posicao e velocidade, porem no modelo de medida do 
    // gps continuei usando xyz apesar de as proprias medidas estarem no referencial NED. 

    // Índices das variáveis de estado
    static constexpr size_t Q0 = 0;     // Quaternion parte 1
    static constexpr size_t Q1 = 1;     // Quaternion parte 2
    static constexpr size_t Q2 = 2;     // Quaternion parte 3
    static constexpr size_t Q3 = 3;     // Quaternion parte 4
    static constexpr size_t WX = 4;     // Velocidade angular X
    static constexpr size_t WY = 5;     // Velocidade angular Y
    static constexpr size_t WZ = 6;     // Velocidade angular Z
    static constexpr size_t PX = 7;     // Posição X (NED)
    static constexpr size_t PY = 8;     // Posição Y (NED)
    static constexpr size_t PZ = 9;     // Posição Z (NED)
    static constexpr size_t VX = 10;    // Velocidade X (NED)
    static constexpr size_t VY = 11;    // Velocidade Y (NED)
    static constexpr size_t VZ = 12;    // Velocidade Z (NED)
    static constexpr size_t AX = 13;    // Aceleração X (NED)
    static constexpr size_t AY = 14;    // Aceleração Y (NED)
    static constexpr size_t AZ = 15;    // Aceleração Z (NED)
    static constexpr size_t ABX = 16;   // Bias acelerômetro X
    static constexpr size_t ABY = 17;   // Bias acelerômetro Y
    static constexpr size_t ABZ = 18;   // Bias acelerômetro Z
    static constexpr size_t GBX = 19;   // Bias giroscópio X
    static constexpr size_t GBY = 20;   // Bias giroscópio Y
    static constexpr size_t GBZ = 21;   // Bias giroscópio Z
    static constexpr size_t MX = 22;    // Campo geomagnético X (NED)
    static constexpr size_t MY = 23;    // Campo geomagnético Y (NED)
    static constexpr size_t MZ = 24;    // Campo geomagnético Z (NED)
    static constexpr size_t MBX = 25;   // Bias magnetômetro X
    static constexpr size_t MBY = 26;   // Bias magnetômetro Y
    static constexpr size_t MBZ = 27;   // Bias magnetômetro Z

    // Métodos de acesso (const) - leitura
    T q0()    const { return (*this)[Q0]; }
    T q1()    const { return (*this)[Q1]; }
    T q2()    const { return (*this)[Q2]; }
    T q3()    const { return (*this)[Q3]; }
    T wx()    const { return (*this)[WX]; }
    T wy()    const { return (*this)[WY]; }
    T wz()    const { return (*this)[WZ]; }
    T px()    const { return (*this)[PX]; }
    T py()    const { return (*this)[PY]; }
    T pz()    const { return (*this)[PZ]; }
    T vx()    const { return (*this)[VX]; }
    T vy()    const { return (*this)[VY]; }
    T vz()    const { return (*this)[VZ]; }
    T ax()    const { return (*this)[AX]; }
    T ay()    const { return (*this)[AY]; }
    T az()    const { return (*this)[AZ]; }
    T abx()   const { return (*this)[ABX]; }
    T aby()   const { return (*this)[ABY]; }
    T abz()   const { return (*this)[ABZ]; }
    T gbx()   const { return (*this)[GBX]; }
    T gby()   const { return (*this)[GBY]; }
    T gbz()   const { return (*this)[GBZ]; }
    T mx()    const { return (*this)[MX]; }
    T my()    const { return (*this)[MY]; }
    T mz()    const { return (*this)[MZ]; }
    T mbx()   const { return (*this)[MBX]; }
    T mby()   const { return (*this)[MBY]; }
    T mbz()   const { return (*this)[MBZ]; }

    // Métodos de acesso (referência) - escrita
    T& q0()   { return (*this)[Q0]; }
    T& q1()   { return (*this)[Q1]; }
    T& q2()   { return (*this)[Q2]; }
    T& q3()   { return (*this)[Q3]; }
    T& wx()   { return (*this)[WX]; }
    T& wy()   { return (*this)[WY]; }
    T& wz()   { return (*this)[WZ]; }
    T& px()   { return (*this)[PX]; }
    T& py()   { return (*this)[PY]; }
    T& pz()   { return (*this)[PZ]; }
    T& vx()   { return (*this)[VX]; }
    T& vy()   { return (*this)[VY]; }
    T& vz()   { return (*this)[VZ]; }
    T& ax()   { return (*this)[AX]; }
    T& ay()   { return (*this)[AY]; }
    T& az()   { return (*this)[AZ]; }
    T& abx()  { return (*this)[ABX]; }
    T& aby()  { return (*this)[ABY]; }
    T& abz()  { return (*this)[ABZ]; }
    T& gbx()  { return (*this)[GBX]; }
    T& gby()  { return (*this)[GBY]; }
    T& gbz()  { return (*this)[GBZ]; }
    T& mx()   { return (*this)[MX]; }
    T& my()   { return (*this)[MY]; }
    T& mz()   { return (*this)[MZ]; }
    T& mbx()  { return (*this)[MBX]; }
    T& mby()  { return (*this)[MBY]; }
    T& mbz()  { return (*this)[MBZ]; }
};

/**
 * @brief System control-input vector-type for a 3DOF planar robot
 *
 * This is the system control-input of a very simple planar robot that
 * can control the velocity in its current direction as well as the
 * change in direction.
 *
 * @param T Numeric scalar type
 */
template<typename T>
class Control : public Kalman::Vector<T, 0>
{
public:
    KALMAN_VECTOR(Control, T, 0)
};

/**
 * @brief System model for a simple planar 3DOF robot
 *
 * This is the system model defining how our robot moves from one 
 * time-step to the next, i.e. how the system state evolves over time.
 *
 * @param T Numeric scalar type
 * @param CovarianceBase Class template to determine the covariance representation
 *                       (as covariance matrix (StandardBase) or as lower-triangular
 *                       coveriace square root (SquareRootBase))
 */
template<typename T, template<class> class CovarianceBase = Kalman::StandardBase>
class SystemModel : public Kalman::LinearizedSystemModel<State<T>, Control<T>, CovarianceBase>
{
public:
    //! State type shortcut definition
	typedef State<T> S;
    
    //! Control type shortcut definition
    typedef Control<T> C;
    
    // Intervalo de tempo da iteração 
    float dt;

    /**
     * @brief Definition of (non-linear) state transition function
     *
     * This function defines how the system state is propagated through time,
     * i.e. it defines in which state \f$\hat{x}_{k+1}\f$ is system is expected to 
     * be in time-step \f$k+1\f$ given the current state \f$x_k\f$ in step \f$k\f$ and
     * the system control input \f$u\f$.
     *
     * @param [in] x The system state in current time-step
     * @param [in] u The control vector input
     * @returns The (predicted) system state in the next time-step
     */
    S f(const S& x, const C& u) const
    {
        //! Predicted state vector after transition
        S x_ = x;

        // Extrair componentes necessarias para a predicao
        T q0 = x.q0();
        T q1 = x.q1();
        T q2 = x.q2();
        T q3 = x.q3();
        T wx = x.wx();
        T wy = x.wy();
        T wz = x.wz();
        T vx = x.vx();
        T vy = x.vy();
        T vz = x.vz();
        T ax = x.ax();
        T ay = x.ay();
        T az = x.az();
        T px = x.px();
        T py = x.py();
        T pz = x.pz();

        // Predicao do quaternion de acordo com a matriz de velocidade angular
        x_.q0() = q0 - 0.5 * dt * (wx * q1 + wy * q2 + wz * q3);
        x_.q1() = q1 + 0.5 * dt * (wx * q0 + wz * q2 - wy * q3);
        x_.q2() = q2 + 0.5 * dt * (wy * q0 - wz * q1 + wx * q3);
        x_.q3() = q3 + 0.5 * dt * (wz * q0 + wy * q1 - wx * q2);

        // Predicao da posicao
        x_.px() = px + vx * dt + 0.5 * ax * dt * dt;
        x_.py() = py + vy * dt + 0.5 * ay * dt * dt;
        x_.pz() = pz + vz * dt + 0.5 * az * dt * dt;

        // Predicao da velocidade
        x_.vx() = vx + ax * dt;
        x_.vy() = vy + ay * dt;
        x_.vz() = vz + az * dt;

        // Os demais elementos do vetor de estados nao recebem atualizacao, seja porque e previsto 
        // que sejam constantes, como os biases e as componentes do campo magnetico da Terra, seja 
        // porque nao se tem uma trajetoria teorica definida, como a as velocidades angulares e a aceleracao

        // Return transitioned state vector
        return x_;
    }
    
protected:
    /**
     * @brief Update jacobian matrices for the system state transition function using current state
     *
     * This will re-compute the (state-dependent) elements of the jacobian matrices
     * to linearize the non-linear state transition function \f$f(x,u)\f$ around the
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
    void updateJacobians( const S& x, const C& u )
    {
        // F = df/dx (Jacobian of state transition w.r.t. the state)
        this->F.setIdentity();

        // Extrair componentes necessarias para o update da rotacao
        T q0 = x.q0();
        T q1 = x.q1(); 
        T q2 = x.q2();
        T q3 = x.q3();
        T wx = x.wx();
        T wy = x.wy();
        T wz = x.wz();
        
        // Derivadas parciais baseadas em Fq, uma matriz 4x4, a diagonal já setada
        // Linha Q0
        this->F(S::Q0, S::Q1) = -0.5 * dt * wx;
        this->F(S::Q0, S::Q2) = -0.5 * dt * wy;
        this->F(S::Q0, S::Q3) = -0.5 * dt * wz;
        // Linha Q1
        this->F(S::Q1, S::Q0) = 0.5 * dt * wx;
        this->F(S::Q1, S::Q2) = 0.5 * dt * wz;
        this->F(S::Q1, S::Q3) = -0.5 * dt * wy;
        // Linha Q2
        this->F(S::Q2, S::Q0) = 0.5 * dt * wy;
        this->F(S::Q2, S::Q1) = -0.5 * dt * wz;
        this->F(S::Q2, S::Q3) = 0.5 * dt * wx;
        // Linha Q3
        this->F(S::Q3, S::Q0) = 0.5 * dt * wz;
        this->F(S::Q3, S::Q1) = 0.5 * dt * wy;
        this->F(S::Q3, S::Q2) = -0.5 * dt * wx;
        
        // Preenchendo a Jacobiana F para quatérnios e velocidades angulares
        this->F(S::Q0, S::WX) = -0.5 * dt * q1;  // dQ0/dWX
        this->F(S::Q0, S::WY) = -0.5 * dt * q2;  // dQ0/dWY
        this->F(S::Q0, S::WZ) = -0.5 * dt * q3;  // dQ0/dWZ

        this->F(S::Q1, S::WX) = 0.5 * dt * q0;   // dQ1/dWX
        this->F(S::Q1, S::WY) = -0.5 * dt * q3;  // dQ1/dWY
        this->F(S::Q1, S::WZ) = 0.5 * dt * q2;   // dQ1/dWZ

        this->F(S::Q2, S::WX) = 0.5 * dt * q3;   // dQ2/dWX
        this->F(S::Q2, S::WY) = 0.5 * dt * q0;   // dQ2/dWY
        this->F(S::Q2, S::WZ) = -0.5 * dt * q1;  // dQ2/dWZ

        this->F(S::Q3, S::WX) = -0.5 * dt * q2;  // dQ3/dWX
        this->F(S::Q3, S::WY) = 0.5 * dt * q1;   // dQ3/dWY
        this->F(S::Q3, S::WZ) = 0.5 * dt * q0;   // dQ3/dWZ

        // Preenchendo a Jacobiana F para posição, velocidade e aceleração
        // As posicoes na diagonal ja foram setadas como identidade
        this->F(S::PX, S::VX) = dt;            // Posição X depende da velocidade X
        this->F(S::PX, S::AX) = 0.5 * dt * dt;  // Posição X depende da aceleração X
        this->F(S::PY, S::VY) = dt;            // Posição Y depende da velocidade Y
        this->F(S::PY, S::AY) = 0.5 * dt * dt;  // Posição Y depende da aceleração Y
        this->F(S::PZ, S::VZ) = dt;            // Posição Z depende da velocidade Z
        this->F(S::PZ, S::AZ) = 0.5 * dt * dt;  // Posição Z depende da aceleração Z

        this->F(S::VX, S::AX) = dt;            // Velocidade X depende da aceleração X
        this->F(S::VY, S::AY) = dt;            // Velocidade Y depende da aceleração Y
        this->F(S::VZ, S::AZ) = dt;            // Velocidade Z depende da aceleração Z

        // W = df/dw (Jacobian of state transition w.r.t. the noise)
        this->W.setIdentity();
        // Esse TODO aqui embaixo vale para mim tambem, nao fiz modelagem do ruido
        // TODO: more sophisticated noise modelling
        //       i.e. The noise affects the the direction in which we move as 
        //       well as the velocity (i.e. the distance we move)
    }
};

#endif