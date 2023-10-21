from sympy import *
import traceback 

def matrix_to_ccode(matrix, name):
    result = "\\\\{} is initialized as a RowMajor matrix\n{} <<\n".format(name,name)
    for i in matrix:
        result = result + "".join(["\t", ccode(i), ",\n"])
    size = len(result)
    result = result[:size-2]
    result = result + ";"
    return result

if __name__ == '__main__':
    init_printing(use_unicode=True)

    try:
        # STATE
        x = Symbol('x')
        y = Symbol('y')
        theta = Symbol('theta')
        l_ratio = Symbol('l_ratio')
        d_ratio = Symbol('d_ratio')
        h = Symbol('h')
        v = Symbol('v')
        w = Symbol('w')
        state = Matrix([x,y,theta,l_ratio,d_ratio,h,v,w])

        l = l_ratio*h
        d = d_ratio*h

        # VIEW MATRIX
        # vr00 = Symbol('Vr_00')
        # vr01 = Symbol('Vr_01')
        # vr02 = Symbol('Vr_02')
        # vr10 = Symbol('Vr_10')
        # vr11 = Symbol('Vr_11')
        # vr12 = Symbol('Vr_12')
        # vr20 = Symbol('Vr_20')
        # vr21 = Symbol('Vr_21')
        # vr22 = Symbol('Vr_22')
        # VR = Matrix([[vr00, vr01, vr02],[vr10, vr11, vr12],[vr20, vr21, vr22]])
        # vtx = Symbol('Vtx')
        # vty = Symbol('Vty')
        # vtz = Symbol('Vtz')
        vr00 = Symbol('vr00')
        vr01 = Symbol('vr01')
        vr02 = Symbol('vr02')
        vr10 = Symbol('vr10')
        vr11 = Symbol('vr11')
        vr12 = Symbol('vr12')
        vr20 = Symbol('vr20')
        vr21 = Symbol('vr21')
        vr22 = Symbol('vr22')
        VR = Matrix([[vr00, vr01, vr02],[vr10, vr11, vr12],[vr20, vr21, vr22]])
        vtx = Symbol('vtx')
        vty = Symbol('vty')
        vtz = Symbol('vtz')
        VT = Matrix([vtx,vty,vtz])
        viewMat = eye(4)
        viewMat[0:3,0:3] = VR
        viewMat[0:3,3] = VT

        # PROJECTION MATRIX
        fx = Symbol('fx')
        fy = Symbol('fy')
        cx = Symbol('cx')
        cy = Symbol('cy')
        P = Matrix([[fx,0,cx,0],[0,fy,cy,0],[0,0,1,0]])

        # WORLD VIEW PROJECTION MATRIX
        m00 = Symbol('m_00')
        m01 = Symbol('m_01')
        m02 = Symbol('m_02')
        m03 = Symbol('m_03')
        m10 = Symbol('m_10')
        m11 = Symbol('m_11')
        m12 = Symbol('m_12')
        m13 = Symbol('m_13')
        m20 = Symbol('m_20')
        m21 = Symbol('m_21')
        m22 = Symbol('m_22')
        m23 = Symbol('m_23')
        M_wvp = Matrix([[m00, m01, m02, m03],[m10, m11, m12, m13],[m20, m21, m22, m23]])


        # ELLIPSOID
        z = Symbol('z')

        a = Symbol('a_3D')
        b = Symbol('b_3D')
        c = Symbol('c_3D')

        subs_ellipsoid = {
            z : h/2,
            a : l/2,
            b : d/2,
            c : h/2
        }

        Q = Matrix([[a**2, 0, 0, 0], [0, b**2, 0, 0],[0, 0, c**2, 0],[0, 0, 0, -1]])

        # WORLD Matrix
        WR = Matrix([[cos(theta), -sin(theta), 0],[sin(theta), cos(theta), 0],[0,0,1]])
        Re_w = eye(4)
        Re_w[0:3,0:3] = WR.T

        center_ellipsoid = Matrix([x,y,z])
        T_c_inv = eye(4)
        T_c_inv[0:3,3] = center_ellipsoid

        W = T_c_inv*Re_w.T
        # world view projection matrix
        M_wvp_real = P*viewMat*W

        subs_wvp = {
            m00 : M_wvp_real[0,0],
            m01 : M_wvp_real[0,1],
            m02 : M_wvp_real[0,2],
            m03 : M_wvp_real[0,3],
            m10 : M_wvp_real[1,0],
            m11 : M_wvp_real[1,1],
            m12 : M_wvp_real[1,2],
            m13 : M_wvp_real[1,3],
            m20 : M_wvp_real[2,0],
            m21 : M_wvp_real[2,1],
            m22 : M_wvp_real[2,2],
            m23 : M_wvp_real[2,3]
        }

        # ELLIPSE
        C_star = M_wvp * Q * M_wvp.T

        C_star = (-1/C_star[2,2])*C_star
        ellipse_centre = Matrix([-C_star[0,2],-C_star[1,2]])

        T_e_origin = eye(3)
        T_e_origin[0:2, 2] = -ellipse_centre
        C_centre = T_e_origin*C_star*T_e_origin.T
        C_centre = 0.5*(C_centre + C_centre.T)

        C_submat = C_centre[0:2, 0:2]
        # Simplify the formula
        # do that on other terms require too much time or do not have effect
        print("Start simplify C_submat...")
        C_submat.simplify()
        print("Done")

        c00 = Symbol('c_00')
        c01 = Symbol('c_01')
        c10 = Symbol('c_10')
        c11 = Symbol('c_11')
        # Substitutions to reduce complexity in computing eigenvalues
        subs_c = {
            c00: C_submat[0,0],
            c01: C_submat[0,1],
            c10: C_submat[1,0],
            c11: C_submat[1,1],
        }

        temp = Matrix([[c00,c01], [c10,c11]])

        lambdas = list(temp.eigenvals().keys())
        lambdas_list = [sqrt(l) for l in lambdas]
        semi_axes = Matrix(lambdas_list)
        # Restore real values
        semi_axes = semi_axes.subs(subs_c)

        eigenvect_list = [i[2] for i in temp.eigenvects()]
        ellipse_rotMat = eye(2)
        ellipse_rotMat[0:2,0] = eigenvect_list[0]
        ellipse_rotMat[0:2,1] = eigenvect_list[1]
        e_theta = atan2(ellipse_rotMat[1,0], ellipse_rotMat[0, 0])
        e_theta = e_theta.subs(subs_c)
        print("Start simplify e_theta...")
        e_theta = e_theta.simplify() # it takes 10 minutes to simplify
        print("Done")


        # [X_center, Y_center, semi_axis_A, semi_axis_B, theta]
        ellipse_state = Matrix([ellipse_centre[0], ellipse_centre[1], semi_axes[0], semi_axes[1], e_theta])

        # Here we can obtain the final equations
        ellipse_state = ellipse_state.subs(subs_wvp).subs(subs_ellipsoid)


        # %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        print("Start simplify ellipse_state...")
        ellipse_state = ellipse_state.simplify()
        print("Done")

        jacobian = ellipse_state.jacobian(state)

        print("Start simplify jacobian...")
        jacobian = jacobian.simplify()
        print("Done")


        with open("outputpy.txt", 'w') as file:
            file.write("Ellipse state\n")
            file.write(pycode(ellipse_state))
            file.write("\n\n")
            file.write("Jacobian\n")
            file.write(pycode(jacobian))

        with open("outputc.txt", 'w') as file:
            file.write("Ellipse state\n")
            file.write(matrix_to_ccode(ellipse_state, "ellipse_state"))
            file.write("\n\n")
            file.write("Jacobian\n")
            file.write(matrix_to_ccode(jacobian, "jacobian"))

    except:
        with open("errorlog.txt", 'w') as file:
            file.write("Exception occurred\n")
            traceback.print_exc(file=file)