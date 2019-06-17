
Data Driven à la mode mais en fait pas tout jeune
=================================================

Pourquoi ce dépôt ?
-------------------

Ce dépôt contient quelques exemples de résolution de problèmes jouets par des approches dites data-driven. L'objectif ici est simplement de montrer que les approches data driven, à la mode ces derniers temps, peuvent en fait simplement se mettre en place à partir de méthodes établies depuis longtemps. 

Installation
------------

Ce projet nécessite un compilateur c++ compatible c++11 et la librairie Eigen. La compilation se réalise via la chaine de compilation SCons::

  scons EIGEN_INC=/my/eigen/header/files/directory

Licence
-------


Ce dépôt utilise la librairie Eigen et également la librairie TinyXml (les sources de TinyXml sont directement incluses dans le projet)


Applications
------------

1 - Aéroélasticité
^^^^^^^^^^^^^^^^^^ 

Le premier exemple traité est un problème d'aéroélasticité à deux degrés de libertés. 

.. image:: images/airfoil_1.png


Le profil NACA est soumis à des actions mécaniques par le biais des deux ressorts et de l'amortisseur et des actions aéro. Les inconnues du problème sont le déplacement vertical du profil, noté :math:`h(t)` et correspondant au flottement, et la rotation :math:`\alpha(t)` correspondant au tangage.

On définit les différents paramètres suivants :

- c : la longueur du profil
- e : l'épaisseur maximale du profil
- :math:`x_c` la position du centre de gravité du profil
- :math:`x_f` la position du centre de rotation du profil
 
Le profil NACA est considéré comme rigide et animé d’un mouvement de
translation verticale :math:`h(t)` et d’une rotation :math:`\alpha (t)`.
L’équation mécanique du mouvement se définit par :

.. math::

   \begin{bmatrix}
   m & S \\ S & I_{\alpha}
   \end{bmatrix}
   \begin{Bmatrix}
   \ddot{h} \\ \ddot{\alpha}
   \end{Bmatrix}
   +
   \begin{bmatrix}
   C_h & 0 \\ 0 & 0
   \end{bmatrix}
   \begin{Bmatrix}
   \dot{h} \\ \dot{\alpha}
   \end{Bmatrix}
   +
   \begin{bmatrix}
   K_h & 0 \\ 0 & K_{\alpha}
   \end{bmatrix}
   \begin{Bmatrix}
   h \\ \alpha
   \end{Bmatrix}
   =
   \begin{Bmatrix}
   0 \\ 0
   \end{Bmatrix}

 L’inertie :math:`I_{\alpha}` se définit par :math:`I_{\alpha} = \frac{1}{3} m (c^2 - 3 c x_f +3 x_f^2)`, :math:`S=m\left(\frac{c}{2} - x_f \right)`.
