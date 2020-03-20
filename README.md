# DIGITALIZING-MATERIAL-COLLATION-FROM-PRE-DEMOLITION-SITES // Software II
## IAAC MRAC 19-20 // Software II // Group 04

The word collation is the collection and ordering of materials.This project defines a new methodology to sort and locate the material of a pre-demolition site to create a more reliable digital library (platform) of the resources available in old buildings. The purpose is to reduce the demolition waste generated by using the obsolete constructions as a mine for future buildings.

### WHAT IS OUR VISION?
Repurposing material in the construction sector is not something new. The churches were built using the materials of the roman forum and colosseum etc. In this case what we do in this project is embedding technology to a method that is being used for centuries.
![4-4](https://user-images.githubusercontent.com/57528373/77206881-59d84200-6af8-11ea-9e4c-cb1e156788ab.jpg)
The diagram below shows the data of materials that is being wasted in construction sites. The point that it has different values for each material as minium and maximum amount shows the lack of documentation and need for a more reliable documentation with digitazing.
![4-5](https://user-images.githubusercontent.com/57528373/77206888-5cd33280-6af8-11ea-8f7d-df42e164bc4e.jpg)
Concrete, masonry, timber and steel are 4 materials that we focus on this project due to their high percentage on construction material waste. There are different methods for “What after?” process for each of them but in any case it is important for each to go to reusing instead of sending them to the landfills or downcycling.
![4-6](https://user-images.githubusercontent.com/57528373/77206889-5e045f80-6af8-11ea-9a5c-a426f86ef533.jpg)
### WHAT DO WE DO?
![4-8](https://user-images.githubusercontent.com/57528373/77206893-5fce2300-6af8-11ea-831c-43de630c862d.jpg)
![4-9](https://user-images.githubusercontent.com/57528373/77206898-60ff5000-6af8-11ea-868d-bc4221ccd358.jpg)
### HOW DOES IT WORK?
![4-11](https://user-images.githubusercontent.com/57528373/77206902-62307d00-6af8-11ea-9bd3-3b1b71c580df.jpg)
### WHAT VALUE DO WE PROVIDE?
![4-13](https://user-images.githubusercontent.com/57528373/77206905-63fa4080-6af8-11ea-9c41-7337138ad895.jpg)
### HARDWARE
![4-14](https://user-images.githubusercontent.com/57528373/77206911-65c40400-6af8-11ea-8701-71ec6f27e8ed.jpg)
### BUILDING INSPECTION
![4-16](https://user-images.githubusercontent.com/57528373/77206912-66f53100-6af8-11ea-84f7-2c040788414d.jpg)
### SORTING
![Group_4_Digitalizing Material_Correction-30](https://user-images.githubusercontent.com/57528373/77207331-6741fc00-6af9-11ea-8f18-e5db2ab1b952.jpg)
Image classifier systems must start with a set of input images from which to learn the features used for classification. For each of our categories, we supply both a collection of close-up and un-angled images to give information about texture, and in-situ images to describe the shapes the forms we expect the material to take (e.g. walls, columns, beams, etc).
![Group_4_Digitalizing Material_Correction-31](https://user-images.githubusercontent.com/57528373/77207339-690bbf80-6af9-11ea-87c7-0f67b6c87d9d.jpg)
Our classifier analyzes images by important feature locations in the image, and creates descriptions of the area around them. The algorithms we use look for the corners between edges in the image, and describe the area using a list of simple binary brightness comparisons in a specified pattern around the feature.
These descriptions are clustered together to find a set of ‘visual words’ that can describe the images of our chosen subject matter. Images can be assigned to a category by analyzing how many of the visual words that correspond to the category also appear in the image. 
Finally, these predictions are weighted  with a simple comparison of the histograms of hue and saturation from the image, compared to the average histograms of each category.
![Group_4_Digitalizing Material_Correction-32](https://user-images.githubusercontent.com/57528373/77207340-6ad58300-6af9-11ea-81c3-55a94b074c0f.jpg)
We use this classification method to localize areas of a specific material in our drone flight images. At its most basic, we divide the image into a grid of patches and apply the classification separately. However, as the system becomes less effective as the patches become smaller, we instead analyze a larger patch window and gradually move it over the image, averaging the resulting values for each smaller patch. 
### LOCATION
From the exploration flight we have two desired outputs, the first one being an octomap from OrbSlam2 for a further autonomous flight and the second one a colored dense point cloud from photogrammetry. The point cloud will be used to produce a 3D environment where all data acquired will be shown.  
![4-23](https://user-images.githubusercontent.com/57528373/77206924-6fe60280-6af8-11ea-8998-52f98e9b79bf.jpg)
![4-28](https://user-images.githubusercontent.com/57528373/77206930-72e0f300-6af8-11ea-9ab5-b5808e7d179a.jpg)
![4-29](https://user-images.githubusercontent.com/57528373/77206935-74aab680-6af8-11ea-98bc-ba957929a9ec.jpg)
![4-30](https://user-images.githubusercontent.com/57528373/77206939-77a5a700-6af8-11ea-866f-b95b5a307c62.jpg)
### MATERIAL / ELEMENT LOCATION
The next images show our experiments using the high quality images from the drone to reconstruct the 3D environment. Two different sets of point clouds were necessary to create the complete point cloud.  The environment was then processed in Cloud Compare to extract the architectural and structural elements from the point cloud. After having all the elements as geometry files, the surfaces or meshes were imported into Grasshopper for further cleaning. The output of the whole process were clean quad mesh elements named as: ceiling, beams, columns, floor and wall.
![4-32](https://user-images.githubusercontent.com/57528373/77206944-7aa09780-6af8-11ea-89ce-f108c6dbf0ef.jpg)
![4-33](https://user-images.githubusercontent.com/57528373/77206948-7d02f180-6af8-11ea-8aa7-20e952b29e49.jpg)
![4-34](https://user-images.githubusercontent.com/57528373/77206959-81c7a580-6af8-11ea-80b8-dac462356e35.jpg)
The next step was to implement the image retrieval from the 3D environment. Each time you select an element such as “floor” you get all the images related in position and rotation to that geometry.
![4-37](https://user-images.githubusercontent.com/57528373/77206969-85f3c300-6af8-11ea-93cb-d3dd5689cc07.jpg)  
![4-38](https://user-images.githubusercontent.com/57528373/77206975-87bd8680-6af8-11ea-929e-6b68a57f8455.jpg)
![4-42](https://user-images.githubusercontent.com/57528373/77206985-89874a00-6af8-11ea-89d0-6039f2d783bc.jpg)

