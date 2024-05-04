#ifndef DA_TSP_INTERFACE_H
#define DA_TSP_INTERFACE_H

#include <vector>
#include <string>
#include <cmath>
#include "Graph.h"

/**
 * @brief Class that represents the interface of the program
 */
class Interface {
public:

    /**
     * @brief Starts the interface by loading the dataset and cleaning the output file
     * @return True if the water supply network was loaded accordingly and the output file was created/resetted
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
     * @brief Prints the options parts in the menu interface along with the highlighted option
     * @param options The vector of options to show in the menu
     * @param choice The option that is currently selected (highlighted)
     */
    void printMenuOptions(const std::vector<std::string> &options, int choice);


    /**
     * @brief Prints the options parts in the menu interface along with the highlighted option, but the second last option is not shown in yellow
     * @param options The vector of options to show in the menu
     * @param choice The option that is currently selected (highlighted)
     */
    void printMenuOptionsNoBottom(const std::vector<std::string> &options, int choice);

    /**
     * @brief Main menu to select which function should be processed
     */
    void mainMenu();

    /**
     * @brief Menu that is presented when exiting the program
     */
    void exitMenu();

    /**
     * @brief Menu used to select which dataset should be loaded into the program
     * @return True if the dataset was loaded correctly, False otherwise
     */
    bool categoryMenu();

    bool extraFullyConnectedMenu();
    bool realWorldMenu();
    bool toyMenu();

private:
    int infoSpacing = 8;
    int width = 80;
    Graph* graph = nullptr;

    double backtrackResult = NAN;
    double heldKarpResult = NAN;
    double doubleMSTResult = NAN;
    double nearestNeighborResult = NAN;
    double christofidesResult = NAN;
    double realWorldResult = NAN;
};

#endif //DA_TSP_INTERFACE_H
