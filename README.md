# Simulation of Flocking Behavior
**Author:** Dario Eugenio Campisi  
**Date:** January 2024  

- [General description](#general-description)
- [Implementation logic](#implementation-logic)
- [Testing strategies](#testing-strategies)
- [Compilation instructions](#compilation-instructions)
- [Interpretation of the results](#interpretation-of-the-results)

## General description
This project is an attempt at implementing **Boids**, a program developed by Craig Reynolds in 1986, using the **SFML** graphics library. The aim of the program is to simulate the behavior of a generic **flock**, that is, a complex system made up of individual entities, the *boids*, which follow specific rules of movement, referred to here as **flight rules**.

<div align="center">
<img src="utility/boid_shape1.png" width="400">

<b>Figure 1:</b> Geometric shape used for the graphical representation of the boids
</div>

### Flight rules
The movement of each boid in the plane results from the following **rules**. To encourage the formation of multiple independent flocks, these rules are applied only to boids considered **neighbors**, that is, boids whose distance does not exceed a certain value.

- **Separation:** causes boids to maintain a minimum distance from one another, thus preventing them from overlapping
- **Alignment:** causes boids to move in the same direction as their neighbors
- **Cohesion:** pushes boids to move toward the center of mass of the flock

These three simple rules give rise to **chaotic macroscopic behaviors**, very similar to those of real flocks. As for the **boundary behavior** of the graphical window, when a boid starts approaching one of the four edges of the window, it turns to avoid it. To achieve this effect, a **margin** has been defined near the edges; when a boid crosses it, its velocity is gradually reversed according to an appropriate **turn factor**.

### Simulation dynamics
When the program starts, an **initial screen** appears, prompting the user to choose the number of boids to be generated. This number is limited to a maximum of 300 to avoid overloading the graphical window and producing undesirable behaviors:

<div align="center">
<img src="utility/initial_screen.png" width="800">

<b>Figure 2:</b> Initial screen
</div>

To start the simulation, the user can click the *start* button or, alternatively, press *Enter*.

During the simulation, the parameters of the flight rules and the boids' vision radius can be adjusted using **sliders**, making it possible to observe in real time how these changes affect the macroscopic movements of the flocks. To quickly restore the default values, simply press the *reset* button:

<div align="center">
<img src="utility/one_flock.png" width="800">

<b>Figure 3:</b> Frame of a simulation with a single orange flock consisting of 173 boids
</div>

As can be seen in **Figure 3**, a **predator** is constantly present in the simulations, distinguished from the boids by its color and size. Its role is to chase the flocks (the **cohesion** rule is therefore applied to it) while exerting a **repulsive** force on nearby boids. In terms of implementation, this force is analogous to **separation**, but stronger.

### Stochastic component
In each simulation, the generation of one, two, or three **different flocks**, distinguished by color, is equally likely. The possible colors of the flocks and the graphical details of the *top bar* are <span style="color:rgb(0, 102, 204)">blue</span>, <span style="color:rgb(0, 153, 0)">green</span>, and <span style="color:rgb(255, 128, 0)">orange</span>.

<div align="center">
<img src="utility/three_flocks.png" width="800">

<b>Figure 4:</b> Frame of a simulation with 230 boids divided into three different flocks
</div>

### Statistics output
Periodically, the program prints the **mean distance** between boids (taking the different flocks into account, if present) and the **mean speed** of all the boids in the simulation, together with their respective **standard deviations**. Example:

```shell
Mean distance: (252.012 +/- 116.827) px 
Mean speed: (2.00593 +/- 0.184291) px/frameTime
```

This functionality is implemented in the `st::printStatistics()` function, contained in the `statistics.hpp` file along with some auxiliary functions for calculating the mean and standard deviation.

**Note:** The *frequency* at which the statistics are printed depends on the performance maintained by the simulation. Some reference values:
- Standard performance (~115 constant fps): ~25 seconds
- Medium-low performance (~70 constant fps): ~40 seconds

## Implementation logic
The program is built around three classes: `bd::Boid`, `sd::Slider`, and `bt::Button`, all strongly based on classes provided by **SFML/Graphics** and contained in `boid.hpp`, `slider.hpp`, and `button.hpp`, respectively.

### bd::Boid
This is the fundamental class of the program: it allows boids to be represented and moved within the window. Internally, it consists of an `sf::ConvexShape`, defined to obtain the shape shown in [Figure 1](#general-description), and an `sf::Vector2f` representing the boid's velocity.

**Constructors**
```cpp
// used for boids
bd::Boid::Boid(const sf::Color &color, const sf::Vector2f &position,
           const sf::Vector2f &velocity)
    : velocity_{velocity} {
  shape_.setPointCount(4);
  shape_.setPoint(0, sf::Vector2f(6.f, 0.f));
  shape_.setPoint(1, sf::Vector2f(-5.f, 4.f));
  shape_.setPoint(2, sf::Vector2f(-1.f, 0.f));
  shape_.setPoint(3, sf::Vector2f(-5.f, -4.f));
  shape_.setOrigin(0.f, 0.f);

  shape_.setFillColor(color);

  shape_.setPosition(position);
}
```

```cpp
// used for the predator
bd::Boid::Boid(const sf::Vector2f &position, const sf::Vector2f &velocity)
    : velocity_{velocity} {
  shape_.setPointCount(4);
  shape_.setPoint(0, sf::Vector2f(8.f, 0.f));
  shape_.setPoint(1, sf::Vector2f(-7.f, 6.f));
  shape_.setPoint(2, sf::Vector2f(-3.f, 0.f));
  shape_.setPoint(3, sf::Vector2f(-7.f, -6.f));
  shape_.setOrigin(0.f, 0.f);

  shape_.setFillColor(sf::Color(204, 0, 0));

  shape_.setPosition(position);
}
```

**Main methods**

- `bool isCloseAndVisible()`

    Checks whether a boid is within the field of vision of the boid on which the method is called;

- `void maxVelocity()`

    Checks the boid's speed and, if it exceeds the specified **maximum speed**, brings it back to the limit;

- `void avoidBoundary()`

    Enforces the [boundary behavior](#flight-rules) described above;

- `bool isFlockMate()`

    Checks whether a boid belongs to the same flock as the boid on which the method is called; this simply consists of comparing the two boids' colors;

### sd::Slider
This class is used to create and operate the sliders mentioned in [simulation dynamics](#simulation-dynamics). Internally, it consists of: an `sf::RectangleShape` representing the slider track, an `sf::CircleShape` representing the slider handle, an `sf::Text` displaying its title, a `float&` linking the slider to a given parameter, and a `const float` storing the parameter's default value.

**Constructor**

```cpp
sd::Slider::Slider(const std::string &title, const sf::Font &font,
               const sf::Vector2f &line_size, const float dot_radius,
               const sf::Vector2f &position, float &parameter,
               const float default_value)
    : line_{line_size},
      dot_{dot_radius},
      parameter_{parameter},
      default_value_{default_value} {
  line_.setOrigin(line_size.x / 2, line_size.y / 2);
  dot_.setOrigin(dot_radius, dot_radius);

  title_.setFont(font);
  title_.setString(title);
  title_.setCharacterSize(15);

  line_.setPosition(position.x, position.y);
  dot_.setPosition(position.x, position.y);
  title_.setPosition(position.x - line_.getSize().x / 2, position.y - 28.f);
}
```

**Main methods**

- `bool mouseIsOver()`

    Checks whether the mouse is over the slider, a necessary condition for moving its handle;

- `void stayInRange()`

    Prevents the handle from moving beyond the ends of the slider track;

- `void work()`

    Handles all slider functionality, including adjustment of the parameter linked to it;

- `void reset()`

    Returns the handle to the center of the track and the parameter to its default value. This is called when the *reset* button shown, for example, in [Figure 4](#stochastic-component) is pressed;

**Note:** The number of sliders was limited to four for aesthetic reasons. There are several other parameters in the program that could potentially be modified, and the `sd::Slider` class is sufficiently versatile. The particular choice made here is entirely arbitrary.

### bt::Button
This is a fairly simple class used to implement the *start* and *reset* buttons described in [simulation dynamics](#simulation-dynamics). Their functionality, however, relies heavily on `sf::Event`, the SFML class that handles events in the graphical window. Internally, the class consists of an `sf::RectangleShape` giving the button its shape and an `sf::Text` representing its label.

**Constructor**
```cpp
bt::Button::Button(const std::string &title, const sf::Font &font,
               const sf::Vector2f &rect_size, unsigned int text_size,
               const sf::Vector2f &position)
    : rect_{rect_size} {
  rect_.setOrigin(rect_.getSize().x / 2, rect_.getSize().y / 2);
  rect_.setPosition(position.x, position.y);

  text_.setFont(font);
  text_.setString(title);
  text_.setCharacterSize(text_size);
  text_.setStyle(sf::Text::Bold);
  text_.setOrigin(text_.getGlobalBounds().getSize() / 2.f +
                  text_.getLocalBounds().getPosition());
  text_.setPosition(position.x, position.y);
}
```

**Main methods**

- `bool mouseIsOver()`
    
    A method analogous to `sd::Slider::mouseIsOver()`;

### Quick overview of main.cpp
`main.cpp` is the program's main file. Its overall structure is summarized below, in order:

- Declaration of the program's **"global" variables**, including the window dimensions, flight-rule parameters, and sliders linked to those parameters;
- Management of **random number generation**, used for the boids' initial positions and velocities;
- Definition of all elements making up the **[initial screen](#simulation-dynamics)**;
- Management of the boids' **[random colors](#stochastic-component)** and construction of the **predator**;
- Management of the **window** and declaration of the *top bar* elements;
- **Game loop**
    - Handling **`sf::Event`** events;
    - **Game loop core**
        - Construction of the boids (performed only once);
        - Calculation of fps;
        - Operation of the sliders;
        - Management of boid and predator movement;
    - **Rendering**;
    - Management of the **[statistics output](#statistics-output)**;
    - Management of the **initial screen**;

### Other additional features

- **Field of view:** To make the simulation more realistic, a **viewing angle** was defined to prevent boids from being aware of what happens "behind their backs." This is implemented within [`bd::Boid::isCloseAndVisible()`](#bdboid). The default value is 250°.  

<a id="left-click-to-generate-a-boid"></a>
- **Left-click to generate a boid:** New boids can be added to those generated at the start of the simulation by **clicking** anywhere in the window, except on the top bar. The initial position of the new boids is that of the mouse cursor, while their initial velocity and color (subject to the number of flocks in the simulation) are random.  
This feature was implemented using `sf::Event::MouseButtonReleased` and `sf::Mouse::Left`. To avoid disturbing the simulation's equilibrium and to maintain acceptable performance, it is advisable to use it *cum grano salis*.  

- **Handling loss of focus:** When the *Boids Simulation* window is not active, the simulation **pauses**. Specifically, the boids stop moving and the window becomes slightly darker.  
To achieve this, a `bool window_in_focus{1}` was defined, whose value depends on the pair of events `sf::Event::GainedFocus` and `sf::Event::LostFocus`. The entire [game loop core](#quick-overview-of-maincpp) is executed only if `window_in_focus == 1`. Furthermore, if `window_in_focus == 0`, an `sf::RectangleShape darkness` the size of the window, colored black with reduced opacity, is drawn over all other window elements.

## Testing strategies
Thanks to the graphical implementation, a considerable part of the program's testing could be carried out by observing the behavior of the various elements in the window: boids, predator, sliders, and buttons.
Nevertheless, the **doctest** testing library proved useful for performing more thorough tests on some of the code's most important functions, particularly:

- `bd::Boid::isCloseAndVisible()`
- `bd::Boid::isFlockMate()`
- `sd::Slider::reset()`
- `st::printStatistics()`

These tests are contained in `boids.test.cpp`. Commands to run them:

```shell
% pwd
/Users/dariocampisi/boids
% cmake -S . -B build
...
% cmake --build build
...
% ./build/boids.t
```

## Compilation instructions
It is recommended to compile and run the program using **CMake**, with the `CMakeLists.txt` file.  
**Important:** The program was developed on macOS using **version 2.6** of SFML, which is automatically installed by Homebrew.

Commands to compile and run it:

```shell
% pwd
/Users/dariocampisi/boids
% cmake -S . -B build
...
% cmake --build build
...
% ./build/boids
```

## Interpretation of the results
The program works reasonably well, and the movement of the boids is generally quite similar to that of a **real flock**.

The sliders perform their function correctly, although they deliberately allow only limited freedom to change their associated parameters (which can be increased or decreased by at most 50%). This is because the [testing](#testing-strategies) phase showed that excessive changes to these parameters inevitably lead to incorrect behavior by the boids. In fact, this issue persists even in the final version of the code, although to a much lesser extent and less frequently. Certain combinations of flight-rule parameters, together with a very large number of boids, may still produce undesirable behavior, especially near the edges of the window. If this happens, it is advisable to use the *reset* button.

As long as the guideline not to use an excessive number of boids is followed (especially when adding them via [left-click](#left-click-to-generate-a-boid)), the program's performance remains consistently good and stable.

As for the statistics printed on screen, they seem consistent with expectations. In particular, the **mean distance** appears to respond appropriately to changes in the flight-rule parameters, while the **mean speed** is always lower than the maximum speed and has a low **standard deviation**.
