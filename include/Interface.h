#ifndef DA_TSP_INTERFACE_H
#define DA_TSP_INTERFACE_H

#include <vector>
#include <string>
#include <cmath>
#include "Graph.h"

/**
 * @brief Structure that represents a statistics about a certain TSP algorithm
 */
struct Statistic {
    std::string algorithm;
    double result;
    double time;
};

/**
 * @brief Class that represents the interface of the program
 */
class Interface {
public:

    /**
     * @brief Starts the interface by loading the dataset
     * @return True if the graph was loaded accordingly
     */
    bool init();

    /**
     * @brief Wrapper for the system call to clear the screen based on the Operating System
     */
    void clearScreen();

    /**
     * @brief Prints the top part of the menu interface box
     */
    void printTop();

    /**
     * @brief Prints the bottom part of the menu interface box
     */
    void printBottom();

    /**
     * @brief Writes the wait message and halts until ENTER is pressed
     */
    void waitInput();

    /**
     * @brief Prints the options parts in the menu interface, formatted for the dataset section, along with the highlighted option
     * @param options The vector of options to show in the menu
     * @param choice The option that is currently selected (highlighted)
     */
    void printDatasetsOptions(const std::vector<std::string> &options, int choice);


    /**
     * @brief Prints the options parts in the menu interface, formatted for the algorithm section, along with the highlighted option
     * @param options The vector of options to show in the menu
     * @param choice The option that is currently selected (highlighted)
     */
    void printAlgorithmOptions(const std::vector<std::string> &options, int choice);

    /**
     * @brief Receives a vertex id from the user, until the one given is valid
     * @return The vertex id given by the user
     */
    int receiveVertexId();

    /**
     * @brief Main menu to select which TSP algorithm should be processed
     */
    void mainMenu();

    /**
     * @brief Menu that is presented when exiting the program
     */
    void exitMenu();

    /**
     * @brief Menu used to select which dataset category should be loaded into the program
     */
    void categoryMenu();

    /**
    * @brief Menu used to select which extra fully connected graph should be loaded into the program
    */
    void extraFullyConnectedMenu();

    /**
    * @brief Menu used to select which real world graph should be loaded into the program
    */
    void realWorldMenu();

    /**
    * @brief Menu used to select which toy graph should be loaded into the program
    */
    void toyMenu();

    /**
    * @brief Displays ordered statistics about all the algorithms that have been run in the current session
    */
    void statistics();

private:
    int infoSpacing = 8;
    int width = 80;
    Graph* graph = nullptr;

    std::vector<Statistic> stats;
};

#endif //DA_TSP_INTERFACE_H
