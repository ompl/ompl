#!/usr/bin/python3

import sympy as sp
sp.init_printing()

# Constraint class template that will be filled in.
template = """class {name:s}Constraint : public ompl::base::Constraint
{{
public:
    {name:s}Constraint() : ompl::base::Constraint({ambientDim:d}, {constraintDim:d})
    {{
    }}

    void function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const override
    {{
{funcCode:s}    }}

    void jacobian(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::MatrixXd> out) const override
    {{
{jacCode:s}    }}
}};
"""

class Constraint:

    def __init__(self, name, n):
        self.name_ = name
        # Generate an array of variables to use.
        self.variables_ = [sp.Symbol("x[{:d}]".format(i), real=True) for i in range(n)]
        self.constraints_ = []

    def __getitem__(self, index):
        """Return the index^th variable."""
        return self.variables_[index]

    def getVars(self):
        """Create a variable vector."""
        return sp.Matrix(self.variables_)

    def getConstraints(self):
        """Create a constraint function vector."""
        return sp.Matrix(self.constraints_)

    def addConstraint(self, f):
        """Add some symbolic function of variables to the list of constraints."""
        self.constraints_.append(f)

    def jacobian(self):
        """Compute the Jacobian of the current list of constraints."""
        return self.getConstraints().jacobian(self.variables_)

    def funcCode(self):
        ss = ""
        for i in range(len(self.constraints_)):
            ss += ' ' * 8
            ss += sp.printing.cxxcode(sp.simplify(self.constraints_[i]),
                                      assign_to="out[{:d}]".format(i))
            ss += "\n"
        return ss

    def jacCode(self):
        ss = ""
        jac = self.jacobian()
        for i in range(jac.shape[0]):
            for j in range(jac.shape[1]):
                ss += ' ' * 8
                ss += sp.printing.cxxcode(sp.simplify(jac[i, j]),
                                          assign_to="out({:d}, {:d})".format(i, j))
                ss += "\n"
        return ss

    def toCode(self):
        return template.format(name=self.name_,
                               ambientDim=len(self.variables_),
                               constraintDim=len(self.constraints_),
                               funcCode=self.funcCode(),
                               jacCode=self.jacCode())

if __name__ == "__main__":
    # Sphere constraint
    s = Constraint("Sphere", 3)
    s.addConstraint(s.getVars().norm() - 1)
    print(s.toCode())

    # Torus constraint
    t = Constraint("Torus", 3)

    outer_radius = 3
    inner_radius = 1

    c = t.getVars()
    c[2] = 0
    torus = (t.getVars() - outer_radius * c / c.norm()).norm() - inner_radius
    t.addConstraint(torus)

    print(t.toCode())
