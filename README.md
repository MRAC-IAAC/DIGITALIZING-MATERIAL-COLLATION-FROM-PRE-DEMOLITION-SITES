# DIGITALIZING-MATERIAL-COLLATION-FROM-PRE-DEMOLITION-SITES // Software II
### Project Description

This entry explains the method used to get points of interest for the mission planner in the second flight of this project:
* http://www.iaacblog.com/programs/digitalizing-material-collation-pre-demolition-sites/

**Area of interest**
![Select Area](./doc/Select_Area_2.jpg)

**Selected points from mesh topology**
![Select Area](./doc/Select_Area.jpg)

#### Requierements

To download the Rhino file click [HERE](https://drive.google.com/drive/folders/1oU48m0Eazm6iBU8ubfr_FgoaLeqjJuxY?usp=sharing)

* **Rhinoceros 6**

Grasshopper Plugins:

* [FroGH](http://www.co-de-it.com/wordpress/frogh.html)
* Pufferfish
* Weaverbird
* Human
* Firefly
* Parakeet

*Tested on Rhinoceros Version 6 SR23 (6.23.20055.13111, 02/24/2020)*

#### Getting Started:

This method makes use of the images and colors stablished in this technology implementation:
* https://github.com/MRAC-IAAC/material_localization

The grasshopper file already provides internalized meshes with the corresponding colors for each element. Personalized geometry will required to run the *material_localization* with the correct inputs.

#### Workflow

In this example file we were using the color *Yellow* to find the area related to wood, this means that for the brick wall, a piece of it was opaque by pieces of wood. A path is implemented to try to get more information about this particular part.

In each element we use the colors in the mesh vertices to test color inclusion by color threshold in RGB mode for:

* [255, 0, 0] Red = Bricks
* [0, 255, 0] Green = Metal
* [0, 0, 255] Blue = Concrete
* [255, 255, 0] Yellow = Wood
* [128, 128, 128] Gray = Other Materials

For the wall the percentage of materials in the element was as following with a threshold of 0.5 for each color:

* Bricks = **48.8%**
* Metal = **7.6%**
* Concrete = **5.3%**
* **Wood = 7.2%**
* Others = **31.1%**

* **1)** Extract the mesh faces based on the selection of neighbor faces indexes for each vertex based on the previous selection of colors (in this case Yellow).
Is possible to tweak the color treshold to adapt it for the best possible fit. The number of points it's related to the size of the selected area *(more research has to be done here to adapt the distance between points to the required overlay of the images to get the best possible outcome for photogrammetry).*

![Threshold](./doc/Area_1.png)
![Threshold2](./doc/Area_2.png)
![Threshold3](./doc/Area_3.png)
![Threshold4](./doc/Area_4.png)
![Threshold5](./doc/Area_5.png)
![Threshold6](./doc/Area_6.png)

* **2)** Combine the face meshes and do a second selection of the largest area.
* **3)** Remesh the resulting mesh outline to get average points from the centers.
![Threshold](./doc/Step_1.jpg)
![Threshold](./doc/Step_2.jpg)

* **4)** Offset the points based on the mesh normals.
![Threshold](./doc/Step_3.jpg)

#### Credits
Digitalizing Material Collation from Pre-Demolition Sites is a project of IaaC, Institute for Advanced Architecture of Catalonia developed in the Masters of Robotics and Advanced Construction in 2019/20 by: Students: Anna Batallé, Irem Yagmur Cebeci, Matthew Gordon, Roberto Vargas Faculty: Angel Munoz, Soroush Garivani
