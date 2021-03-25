# Scaling the World

Bullet physics works best for objects larger than 0.1 simulation units. This is important for us becasue many real soft robots are on the order of 0.1m in length, leaving us no room to discretize them into smaller links. We must scale the world up to avoid numerical precision errors in bullet.

## Bullet physics guidelines
Unfortunately the bullet physics wiki has been down for over a year now, so we must use an archived version of the page [HERE](https://web.archive.org/web/20170713085948/http://www.bulletphysics.org/mediawiki-1.5.8/index.php/Scaling_The_World). In addition, there is a small typo in that wiki that makes a big difference in how we scale inertias as discussed [HERE](https://pybullet.org/Bullet/phpBB3/viewtopic.php?f=9&t=4160&p=15526#p15516). We use the convention agreed upon in the forum post.

## Standard scaling laws
If we scale all lengths by **X**, we need to correct other variables:

| **_Property_**       |      **_Scale_** | **_Formula_**                   |
|----------------------|-----------------:|---------------------------------|
| **Angle**            |                1 | Theta_sim = 1.0 \* Theta_real   |
| **Angular Velocity** |                1 | w_sim = 1.0 \* w_real           |
| **Length**           |                X | L_sim = X \* L_real             |
| **Linear Velocity**  |                X | v_sim = X \* v_real             |
| **Gravity**          |                X | g_sim = X \* g_real             |
| **Forces**           |                X | F_sim = X \* F_real             |
| **Torques**          | X<sup>2</sup> | T_sim = X<sup>2</sup> \* T_real |
| **Inertias**         | X<sup>2</sup> | I_sim = X<sup>2</sup> \* I_real |

In addition, we could scale masses by a factor **Y**, leading to more corrections:

| **_Property_**       |      **_Scale_** | **_Formula_**                   |
|----------------------|-----------------:|---------------------------------|
| **Forces**           | Y | F_sim = Y\*F_real |
| **Torques**          | Y | T_sim = Y\*T_real |
| **Inertias**         | Y | I_sim = Y\*I_real |

Combining length and mass scaling, we get combined corrections:

| **_Property_**       |      **_Scale_** | **_Formula_**                   |
|----------------------|-----------------:|---------------------------------|
| **Forces**           | X\*Y | F_sim = X\*Y\*F_real |
| **Torques**          | X<sup>2</sup>\*Y | T_sim = X<sup>2</sup>\*Y\*T_real |
| **Inertias**         | X<sup>2</sup>\*Y | I_sim = X<sup>2</sup>\*Y\*I_real |

_Note: We could choose a constant density, thus setting Y=X<sup>3</sup>. However, we do not actually need to do mass scaling according to the forum post from above, so we chose to set Y=1 for simplicity._

We set the sizes of objects in our world according to these units in the various URDF and actuator definition files, and it's up to us to scale these dimensions accordingly. We also need to correct the gravitational constant when setting up our simulation. All other forces, torques, etc are calculated internally.

## SoMo-specific scaling
Since we are discretizing the soft robots into rigid links with angular stiffness and damping terms, we need to correct these terms for the dimensional scale.

```
# Preserve Torque Scaling:
# Rotational Springs:
          T =           K * (Theta-Theta_0)
X^2 * Y * T = X^2 * Y * K * (Theta-Theta_0)

# Rotational Dampers:
          T =           B * (w-w_0)
X^2 * Y * T = X^2 * Y * B * (w-w_0)

```

Therefore, we get the following scaling laws for rotational springs and dampers:

| **_Property_**       |      **_Scale_** | **_Formula_**                   |
|----------------------|-----------------:|---------------------------------|
| **Spring Stiffness**            |                X<sup>2</sup> | K_sim = X<sup>2</sup>\*K_real  |
| **Damping**            |                X<sup>2</sup> | B_sim = X<sup>2</sup>\*B_real  |


We set these values in the joint definition file for each joint.

## Scaling data back to real units
Since the simulations run with a certain length scaling, X, and mass scaling, Y, we need to scale the output data back to real units. Doing this is easy, just inverting all the relationships from above.

| **_Property_**       |      **_Scale_** | **_Formula_**                   |
|----------------------|-----------------:|---------------------------------|
| **Angle**            |                1 | Theta_real= Theta_sim  |
| **Angular Velocity** |                1 | w_real = w_sim          |
| **Length**           |                1/X | L_real = (1/X)\*L             |
| **Linear Velocity**  |                1/X | v_real = (1/X)\*v_sim            |
| **Gravity**          |                1/X | F_real = (1/X)\*F_sim            |
| **Forces**           |                1/X | F_sim = X \* F_real             |
| **Torques**          | 1/X<sup>2</sup> | T_real = (1/X<sup>2</sup>)\*T_sim |
| **Inertias**         | 1/X<sup>2</sup> | I_real = (1/X<sup>2</sup>)\*I_sim |

