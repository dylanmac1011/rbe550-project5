#ifndef DYNAMICS_H

#include <vector>

class Dynamics 
{
public:
    Dynamics(const unsigned int stateDim, const unsigned int controlDim) 
        : stateDim_(stateDim)
        , controlDim_(controlDim)
        {}
    virtual ~Dynamics() = default;

    virtual void computeDerivatives(
        const std::vector<double>& x,
        const std::vector<double>& u,
        std::vector<double>& xdot
    ) const = 0;

    unsigned int stateDim() const { return stateDim_; };
    unsigned int controlDim() const { return controlDim_; };

private:
    const unsigned int stateDim_;
    const unsigned int controlDim_;
};

class PandaDynamics : public Dynamics 
{
public:
    PandaDynamics()
        : Dynamics(14, 7)
        {}

    void computeDerivatives(
        const std::vector<double>& x,
        const std::vector<double>& u,
        std::vector<double>& xdot
    ) const override;
};


#endif // DYNAMICS_H