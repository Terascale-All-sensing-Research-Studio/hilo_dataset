from pathlib import Path
import os
import json


class DataHandler():
    def __init__(
            self,
            root="..\\",
            data_path=str(Path(__file__).resolve().parent / "hi-lo-scene-data.json")  # make relative to repo
    ):
        self._root = os.path.abspath(root)
        if data_path is not None:
            self._data = json.load(open(data_path, "r"))

    @property
    def allScenes(self):
        """ return a list of all scene names/IDs """
        return self._data["allSceneNames"].copy()

    @property
    def allShells(self):
        """ return a list of all shell name lists """
        return self._data["allShellNamesLists"].copy()

    @property
    def allSceneObjectLists(self):
        """ return a list of all object lists """
        return self._data["allScenesObjects"].copy()

    @property
    def allObjects(self):
        """ return a list of all objects """
        return self._data["allObjectsList"].copy()

    @property
    def numObjectsPerScene(self):
        """ return the number of objects in each scene """
        return self._data["numObjectsPerScene"]

    @property
    def numObjects(self):
        """ return the number of objects across all categories """
        assert sum([len(l) for l in self._data["objectsInEachCategory"].copy()]) == len(
            self._data["allObjectsList"].copy())
        return len(self._data["allObjectsList"].copy())

    @property
    def numShellsPerScene(self):
        """ return the number of shells for each scene """
        return len(self._data["shellDistances"])

    @property
    def numClutterLevelsPerScene(self):
        """ return the number of shells for each scene """
        return self._data["numClutterLevelsPerScene"]

    @property
    def numImagesPerImageSet(self):
        """ return the number of images in each image set """
        return self._data["numImagesPerImageSet"]

    @property
    def numDegreesPerArc(self):
        """ return the robot arc length in degrees """
        return self._data["numDegreesPerArc"]

    @property
    def shellDistances(self):
        """ 
        return the distances (inches) each shell should
        be from the turntable center. This is also arc radius.
        """
        return self._data["shellDistances"]

    @property
    def arcEndDegree(self):  # maximum arc start point
        """ return the maximum angle along the arc """
        return self._data["maxArcAngle"]

    @property
    def arcStartDegree(self):  # minimum arc start point
        """ return the minimum angle along the arc """
        return self.arcEndDegree - self.numDegreesPerArc

    @property
    def numImageSetsPerArc(self):
        """ return the number of image sets taken along each arc """
        return self._data["numImageSetsPerArc"]

    @property
    def numDegreesPerImageSetOnArc(self):
        """ return the number of degrees between image sets along each arc """
        return self._data["numDegreesPerImageSetOnArc"]

    @property
    def numImageSetsPerShell(self):
        """ return the number of image sets taken in a single shell """
        return self.numRotationsForAScene * self.numImageSetsPerArc

    @property
    def numRotationsForAScene(self):
        """ return the number of arcs for each shell """
        return self._data["numRotationsForAScene"]

    @property
    def numDegreesPerImageOnArc(self):
        """ returns number of degrees between images on an arc """
        numDegreesPerArc = self.numDegreesPerArc
        numImageSetsPerArc = self.numImageSetsPerArc
        numDegreesPerImageOnArc = numDegreesPerArc / numImageSetsPerArc
        # assert numDegreesPerImageOnArc == self.numDegreesPerImageSetOnArc, "numDegreesPerImageOnArc != numDegreesPerImageSetOnArc"
        return numDegreesPerImageOnArc

    @property
    def numDegreesPerRotation(self):
        """ returns number of degrees for each rotation """
        numRotationsForAScene = self.numRotationsForAScene
        return 360 / numRotationsForAScene

    @property
    def numScenes(self):
        """ returns number of scenes in total in the dataset """
        return self._data["numScenes"]

    @property
    def cameraResolutions(self):
        """ 
        returns camera resolutions in the following order:
        [rs_color, rs_depth, kcolor, kdepth]
        """
        return [[1280, 720], [480, 270], [1920, 1080], [640, 576]]

    def categoryTitleFromIndex(self, index=None, ):
        """ returns category title at index """
        if index is not None:
            return self._data["categoryTitles"].copy()[index]
        return self._data["categoryTitles"].copy()

    def categoryIndexFromTitle(self, title=None, ):
        """ returns index of category named <title> """
        if title is not None:
            return self._data["categoryTitles"].copy().index(title)
        return title

    def objectsInCategoryFromIndex(self, index=None, ):
        """ returns objects in category at <index> """
        if index is not None:
            return self._data["objectsInEachCategory"].copy()[index]
        return self._data["objectsInEachCategory"].copy()

    def numObjectsInCategoryFromIndex(self, index=None, ):
        """ returns number of objects in category at <index> """
        if index is not None:
            return len(self._data["objectsInEachCategory"].copy()[index])
        return self.numObjects

    def getShellNamesForScene(self, sceneName=None, ):
        """ returns list of shell names for scene """
        assert sceneName in self.allScenes, "scene name not found in list of names"
        idx = self.allScenes.index(sceneName)
        return self.allShells[idx]

    def getObjectsForScene(self, sceneName=None, ):
        """ returns a list of objects in scene """
        assert sceneName in self.allScenes, "scene name not found in list of names"
        idx = self.allScenes.index(sceneName)
        return self.allSceneObjectLists[idx]

    def getShellFullName(
            self,
            sceneName,
            shellIndex,
            clutterIndex,
            path=False,
    ):
        """
        returns the full scene name (capture header)
        based on which scene, shell, and L/R according to:

        <sceneID>-shell<shellID>-c<clutterID>-<R/L>

        where sceneID is a 4-digit unique ID
        where shellID indicates the camera distance of the image shell [0,numShellsPerScene]
        """
        assert sceneName in self.allScenes, "scene name not found in list of names"
        assert shellIndex <= self.numShellsPerScene, "shell index exceeds maximum of {}".format(self.numShellsPerScene)
        assert shellIndex >= 0, "shell index cannot be negative"
        # assert clutterIndex >= int(0), "clutter index cannot be negative"
        # shell_name = self.getShellNamesForScene(sceneName=sceneName,)[shellIndex]
        shell_names = self.getShellNamesForScene(sceneName=sceneName, )
        # assert clutterIndex <= len(shell_names[0]), "clutter index exceeds maximum of {}".format(len(shell_names[0]))
        shell_name = shell_names[shellIndex][clutterIndex]

        if path:
            return os.path.join(self._root, sceneName, shell_name)
        return shell_name
