��Y      }�(�
kinematics��
dill._dill��_create_function���(h�_create_code���(C 


�KK K KKKBL  � | \  }}}}}}}}}	}
}}}t          |
�  �        }t          |
�  �        }||z  }|
|z   }||z  }t          ||z  ||z  g|t          |�  �        z  |z   |t          |�  �        z  |z   gg�  �        t          ddg||gg�  �        t          |
g|gg�  �        gS �NK ���sin��cos��array���(�	_Dummy_22��m1��m2��l1��l2��r1��r2��I1��I2��g��q1��q2��q1dot��q2dot��x0��x1��x2��x3��x4�t��<lambdifygenerated-1>��_lambdifygenerated�h#KCˀ �@I�=�R��R��R��R��Q��B��u�	�R���B�	�R���B�	�B��B�	�b��B�	�B��B��B�r�E�2�b�5�>�B�s�2�w�w�J��O�R��B���Z�"�_�#E�F�G�G��QR�TU�PV�Y[�]_�X`�Oa�Ib�Ib�di�ln�ko�rt�qu�jv�dw�dw�x�x�C �))t�R�}��__name__�Nsh#NNt�R�}�}�(�__doc__�X�  Created with lambdify. Signature:

func(arg_0)

Expression:

[Matrix([ [                  r1*sin(q1),                   r1*cos(q1)],...

Source code:

def _lambdifygenerated(_Dummy_22):
    [m1, m2, l1, l2, r1, r2, I1, I2, g, q1, q2, q1dot, q2dot] = _Dummy_22
    x0 = sin(q1)
    x1 = cos(q1)
    x2 = l1*x0
    x3 = q1 + q2
    x4 = l1*x1
    return [array([[r1*x0, r1*x1], [r2*sin(x3) + x2, r2*cos(x3) + x4]]), array([[0, 0], [x2, x4]]), array([[q1], [x3]])]


Imported modules:

��__annotations__�}�u��bh((�cos��numpy.core._multiarray_umath��cos����sin�h4�sin����array��numpy��array���u0�dynamics�h(h(C0 











�KK K K$KKB�  � | \  }}}}}}}}}	}
}}}t          |
�  �        }||dz  z  }t          |
�  �        }||z  }|
|z   }t          |�  �        }||z  }||z   }||z  }t          |�  �        }||z  }| |z
  }||z  }||z  }|||z  z
  ||z  z   }||dz  z  }d|z  }d|z  }||z  } | |z  | z
  }!d|z  |!z  }"d|z  }#t          ||z   ||dz  z  z   ||dz  z  z   |dz  |z  z   ||dz  z  z   |g||||dz  z  z   ||dz  z  z   gg�  �        t          |"|#|| d|z  z
  z  || d|z  z
  z  z   z  z   |!|#z  |"z   g|#||z  |z  | z   z  dgg�  �        t          |	 |z  |z  |z  |	|z  |z  z   g|	 |z  |z  gg�  �        gS �(NKG?�      K t�h
hh��(�	_Dummy_23�hhhhhhhhhhhhhhhhhh �x5��x6��x7��x8��x9��x10��x11��x12��x13��x14��x15��x16��x17��x18��x19��x20��x21�t��<lambdifygenerated-2>�h#h#KBm  � �@I�=�R��R��R��R��Q��B��u�	�R���B�	�B��E��B�	�R���B�	�B��B�	�b��B�	�R���B�	�B��B�	�b��B�	�B��B�	�R���B�
�R�%�C��$��)�C�
�R�%�C�
�S�&�C�
�s�3�w�,��R��
�C�
�R��U�(�C�
�C�%�C�
�B�$�C�
�c�'�C��$�s�(�S�.�C���+�c�/�C���+�C��B��G�b��a��i�'�"�R��U�(�2�R��U�2�X�=��2�q�5��H�#�N�QT�VX�[^�_a�cd�_d�[d�Vd�gj�km�op�kp�gp�Vp�Pq�r�s�s�uz�  ~A�  DG�  IL�  OR�  NR�  UV�  WY�  UY�  NY�  IZ�  ]`�  cf�  bf�  ij�  km�  im�  bm�  ]n�  In�  Do�  ~o�  qt�  ux�  qx�  {~�  q~�  }�  BE�  GI�  JM�  GM�  NP�  GP�  SV�  GV�  BW�  YZ�  A[�  |\�  v]�  v]�  _d�  hi�  gi�  jl�  gl�  mo�  go�  pr�  gr�  uv�  wy�  uy�  z}�  u}�  g}�  f~�  BC�  AC�  DF�  AF�  GJ�  AJ�  @K�  eL�  _M�  _M�  N�  N�h%))t�R�}�h)Nsh#NNt�R�}�}�(h.X�  Created with lambdify. Signature:

func(arg_0)

Expression:

[Matrix([ [I1 + I2 + m1*r1**2*sin(q1)**2 + m1*r1**2*cos(q1)**2 +...

Source code:

def _lambdifygenerated(_Dummy_23):
    [m1, m2, l1, l2, r1, r2, I1, I2, g, q1, q2, q1dot, q2dot] = _Dummy_23
    x0 = sin(q1)
    x1 = m1*r1**2
    x2 = cos(q1)
    x3 = l1*x2
    x4 = q1 + q2
    x5 = cos(x4)
    x6 = r2*x5
    x7 = x3 + x6
    x8 = l1*x0
    x9 = sin(x4)
    x10 = r2*x9
    x11 = -x10 - x8
    x12 = m2*x7
    x13 = m2*x11
    x14 = I2 - x10*x13 + x12*x6
    x15 = m2*r2**2
    x16 = 2*x10
    x17 = 2*x6
    x18 = x13*x17
    x19 = -x12*x16 - x18
    x20 = (1/2)*q2dot*x19
    x21 = (1/2)*q1dot
    return [array([[I1 + I2 + m2*x11**2 + m2*x7**2 + x0**2*x1 + x1*x2**2, x14], [x14, I2 + x15*x5**2 + x15*x9**2]]), array([[x20 + x21*(x12*(-x16 - 2*x8) + x13*(-x17 - 2*x3)), x19*x21 + x20], [x21*(m2*x16*x7 + x18), 0]]), array([[-g*m1*r1*x0 + g*m2*x11], [-g*m2*x10]])]


Imported modules:

�h0}�u��bhZ(�cos�h6�sin�h9�array�h=u0u.