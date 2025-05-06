#include <iostream>
#include <vector>
#include <string>
#include <queue>
#include <map>
#include <iomanip>

using namespace std;

vector<string> nodes;
map<string, string> nodeToStation;
vector<string> allStations;
vector<pair<string, char>> AllStations;
map<string, int> idToStation;
map<string, int> nodeIDToIndex;
map<string, vector<int>> stationNameToAllNodeIndexes;

class Ticket
{
public:
    string source;
    string destination;
    vector<string> shortestPath;
    int distance;
    double price;

    Ticket(const string &src, const string &dest, const vector<string> &path, int dist, double cost)
        : source(src), destination(dest), shortestPath(path), distance(dist), price(cost) {}

    void printTicket() const
    {
        cout << "                                                                                                                                  \n";
        cout << "                                                                                                                                    \n";
        cout << "-----------------------------------------------------------------------------------------------------------------------------------\n";
        cout << "                                                                                                                                    \n";
        cout << "                                                                                                                                    \n";
        cout << "\n\t\t\t--- TICKET ---" << endl;
        cout << "Source Station: " << source << endl;
        cout << "Destination Station: " << destination << endl;
        cout << "Shortest Path: ";
        for (size_t i = 0; i < shortestPath.size(); ++i)
        {
            cout << shortestPath[i];
            if (i < shortestPath.size() - 1)
                cout << " -> ";
        }
        cout << endl;
        cout << "Distance: " << distance << " KM" << endl;
        cout << "Price: Rs. " << price << endl;
        cout << "\t\t\t---------------\n"
             << endl;
        cout << "                                                                                                                                    \n";
        cout << "                                                                                                                                    \n";
        cout << "-----------------------------------------------------------------------------------------------------------------------------------\n";
        cout << "                                                                                                                                    \n";
        cout << "                                                                                                                                    \n";
    }
};

void addNodes(const vector<string> &line, char prefix)
{
    int x = allStations.size();
    for (int i = 0; i < line.size(); ++i)
    {
        string nodeID = string(1, prefix) + to_string(i);
        nodes.push_back(nodeID);
        nodeToStation[nodeID] = line[i];
        allStations.push_back(line[i]);
        AllStations.push_back({line[i], prefix});
        idToStation[line[i]] = x;
        nodeIDToIndex[nodeID] = x;
        stationNameToAllNodeIndexes[line[i]].push_back(x);
        x++;
    }
}

vector<vector<int>> adjMatrix;

void initializeAdjMatrix()
{
    int n = nodes.size();
    adjMatrix.resize(n, vector<int>(n, 0));
}

// void connect(int i, int j) {
//     adjMatrix[i][j] = 1;
//     adjMatrix[j][i] = 1; // undirected
// }

void constructLps(string &pat, vector<int> &lps)
{

    // len stores the length of longest prefix which
    // is also a suffix for the previous index
    int len = 0;

    // lps[0] is always 0
    lps[0] = 0;

    int i = 1;
    while (i < pat.length())
    {

        // If characters match, increment the size of lps
        if (pat[i] == pat[len])
        {
            len++;
            lps[i] = len;
            i++;
        }

        // If there is a mismatch
        else
        {
            if (len != 0)
            {

                // Update len to the previous lps value
                // to avoid reduntant comparisons
                len = lps[len - 1];
            }
            else
            {

                // If no matching prefix found, set lps[i] to 0
                lps[i] = 0;
                i++;
            }
        }
    }
}

bool search(string s)
{
    int m = s.length();
    vector<int> lps(m);

    // Construct the LPS array for the input string
    constructLps(s, lps);

    // Number of columns in the grid
    const int columns = 3;
    int count = 0;

    cout << "\nSuggested Stations:\n";
    cout << "------------------------------------------------------------\n";

    // Traverse through all station names
    for (int k = 0; k < allStations.size(); k++)
    {
        int i = 0, j = 0;
        int n = allStations[k].size();
        string txt = allStations[k];

        // Perform KMP string matching
        while (i < n)
        {
            if (txt[i] == s[j])
            {
                i++;
                j++;

                // If the entire pattern is matched
                if (j == m)
                {
                    // Print the station in grid format
                    cout << setw(5) << idToStation[allStations[k]] << " " << setw(25) << allStations[k] << "\n";
                    count++;

                    // Add a newline after every 'columns' stations
                    if (count % columns == 0)
                    {
                        cout << endl;
                    }
                    break;
                }
            }
            else
            {
                if (j != 0)
                {
                    j = lps[j - 1];
                }
                else
                {
                    i++;
                }
            }
        }
    }

    // Add a newline if the last row is incomplete
    if (count % columns != 0)
    {
        cout << endl;
    }

    cout << "------------------------------------------------------------\n";

    // Return true if matches were found, otherwise false
    return count > 0;
}

// Modified connect function to take weight (default is 1)
void connect(int i, int j, int weight = 1)
{
    adjMatrix[i][j] = weight;
    adjMatrix[j][i] = weight; // undirected
}

// Dijkstra algorithm using adjacency matrix
void dijkstra(int src, vector<int> &dist, vector<int> &prev)
{
    int n = adjMatrix.size();
    dist.assign(n, INT_MAX);
    prev.assign(n, -1);
    vector<bool> visited(n, false);

    dist[src] = 0;

    for (int count = 0; count < n - 1; ++count)
    {
        int u = -1;
        int minDist = INT_MAX;
        for (int i = 0; i < n; ++i)
            if (!visited[i] && dist[i] < minDist)
            {
                minDist = dist[i];
                u = i;
            }

        if (u == -1)
            break;
        visited[u] = true;

        for (int v = 0; v < n; ++v)
            if (!visited[v] && adjMatrix[u][v] > 0 && dist[u] + adjMatrix[u][v] < dist[v])
            {
                dist[v] = dist[u] + adjMatrix[u][v];
                prev[v] = u;
            }
    }
}

// Used in multi trip planning
int minDistancee(string srcStation, string destStation)
{
    if (stationNameToAllNodeIndexes.find(srcStation) == stationNameToAllNodeIndexes.end() or stationNameToAllNodeIndexes.find(destStation) == stationNameToAllNodeIndexes.end())
    {
        cout << "Invalid station name.\n";
        return 0;
    }

    vector<int> srcIndices = stationNameToAllNodeIndexes[srcStation];
    vector<int> destIndices = stationNameToAllNodeIndexes[destStation];

    int minDistance = INT_MAX;

    for (int srcIndex : srcIndices)
    {
        vector<int> dist, prev;
        dijkstra(srcIndex, dist, prev);

        for (int destIndex : destIndices)
        {
            if (dist[destIndex] < minDistance) {
                minDistance = dist[destIndex];
            }
        }
    }

    if (minDistance == INT_MAX) {
        cout << "No path found between stations.\n";
        return 0;
    }

    return minDistance;
}

bool allVisited(vector<bool> &vis)
{
    for (bool v : vis)
    {
        if (!v)
            return false;
    }
    return true;
}

int tsp(string s, vector<bool> &vis, vector<string> &locToVisit)
{
    if (allVisited(vis))
    {
        return minDistancee(s, locToVisit[0]); // Return to the starting station
    }

    int mini = INT_MAX;
    for (int i = 1; i < locToVisit.size(); i++)
    {
        if (!vis[i])
        {
            vis[i] = true;
            mini = min(mini, minDistancee(s, locToVisit[i]) + tsp(locToVisit[i], vis, locToVisit));
            vis[i] = false;
        }
    }
    return mini;
}

void multiTripPlanner()
{
    int k;
    cout << "Enter number of stations to visit (max 7): ";
    cin >> k;

    // Clear the input buffer
    cin.ignore(numeric_limits<streamsize>::max(), '\n');

    vector<string> locToVisit;
    for (int i = 0; i < k; ++i)
    {
        string partialName;
        cout << "Enter partial name of station " << i + 1 << ": ";
        getline(cin, partialName);

        // Use the search function to find matching stations
        search(partialName);

        int stationID;
        cout << "Select station ID: ";
        cin >> stationID;

        // Clear the input buffer again
        cin.ignore(numeric_limits<streamsize>::max(), '\n');

        // Validate station ID
        if (stationID < 0 || stationID >= allStations.size())
        {
            cout << "Invalid station ID. Please try again.\n";
            --i; // Retry the current station input
            continue;
        }

        // Add the selected station to the list
        locToVisit.push_back(allStations[stationID]);
    }

    vector<bool> vis(locToVisit.size(), false);
    vis[0] = true; // Mark the starting station as visited

    int result = tsp(locToVisit[0], vis, locToVisit);
    cout << "Minimum distance to visit all stations: " << result << " KM" << endl;
    cout << "Average Time Of Travel: " << result * 60.0 / 50 << " min" << endl;
}

vector<vector<int>> multiSourceBFS(const vector<int> &sources)
{
    int n = adjMatrix.size();
    
    // Store distances for each source
    vector<vector<int>> distances(sources.size(), vector<int>(n, INT_MAX)); 
    queue<pair<int, int>> q;  // {current station, distance from source}

    // Perform BFS for each source
    for (int srcIndex = 0; srcIndex < sources.size(); ++srcIndex)
    {
        int src = sources[srcIndex];
        distances[srcIndex][src] = 0;
        q.push({src, 0});

        while (!q.empty())
        {
            int current = q.front().first;
            int dist = q.front().second;
            q.pop();

            // Traverse neighbors
            for (int neighbor = 0; neighbor < n; ++neighbor)
            {
                if (adjMatrix[current][neighbor] > 0)
                { 
                    // If there's a connection, update the distance
                    int newDist = dist + adjMatrix[current][neighbor];
                    if (newDist < distances[srcIndex][neighbor])
                    {
                        distances[srcIndex][neighbor] = newDist;
                        q.push({neighbor, newDist});
                    }
                }
            }
        }
    }

    return distances;
}

int findBestMeetingPoint(const vector<int> &sources)
{
    vector<vector<int>> distances = multiSourceBFS(sources);

    int n = adjMatrix.size();
    int minMaxDistance = INT_MAX;
    int minTotalDistance = INT_MAX;
    int bestStation = -1;

    // Iterate over all stations
    for (int station = 0; station < n; ++station)
    {
        int maxDistance = 0;
        int totalDistance = 0;
        bool reachable = true;

        // Calculate max and total distance for this station
        for (int srcIndex = 0; srcIndex < sources.size(); ++srcIndex)
        {
            int dist = distances[srcIndex][station];
            if (dist == INT_MAX) {
                reachable = false; // If any source cannot reach this station, skip it
                break;
            }
            maxDistance = max(maxDistance, dist);
            totalDistance += dist;
        }

        if (!reachable) continue;

        // Update the best station
        if (maxDistance < minMaxDistance or (maxDistance == minMaxDistance and totalDistance < minTotalDistance)) {
            minMaxDistance = maxDistance;
            minTotalDistance = totalDistance;
            bestStation = station;
        }
    }

    return bestStation;
}

// Helper to reconstruct path
vector<string> getPath(int src, int dest, const vector<int> &prev)
{
    vector<string> path;
    for (int at = dest; at != -1; at = prev[at]) {
        path.push_back(nodeToStation[nodes[at]]);
    }
        
    reverse(path.begin(), path.end());
    return path;
}

void addInterchange(const string &stationName)
{
    auto &ids = stationNameToAllNodeIndexes[stationName];
    for (int i = 0; i < ids.size(); ++i)
    {
        for (int j = i + 1; j < ids.size(); ++j)
        {
            // Add small weight for transfers
            connect(ids[i], ids[j], 2); 
        }
    }
}

// User-friendly function to get shortest path
void getShortestPaths(string srcStation, string destStation)
{
    if (idToStation.find(srcStation) == idToStation.end() or idToStation.find(destStation) == idToStation.end())
    {
        cout << "Invalid station name.\n";
        return;
    }

    int srcIndex = idToStation[srcStation];
    int destIndex = idToStation[destStation];

    vector<int> dist, prev;
    dijkstra(srcIndex, dist, prev);
    vector<string> path = getPath(srcIndex, destIndex, prev);
    cout << "-----------------------------------------------------------------------------------------------------------------------------------\n";
    cout << "                                                                                                                                    \n";
    cout << "                                                                                                                                    \n";

    cout << "Shortest path from '" << srcStation << "' to '" << destStation << "':\n";
    // cout<<path[0];
    // for(int i=1;i<path.size();i++){
    //     if(idToStations[path[1]] > idToStation[path[0]]){
    //         cout<<"Towards : "<<
    //     }
    // }
    for (const auto &station : path)
    {
        cout << station;
        if (station != destStation)
            cout << " -> ";
    }
    cout << "\nDistance: " << dist[destIndex] << "\n";
    cout << "                                                                                                                                    \n";
    cout << "                                                                                                                                    \n";
    cout << "-----------------------------------------------------------------------------------------------------------------------------------\n";
}

void getShortestPath(string srcStation, string destStation)
{
    if (stationNameToAllNodeIndexes.find(srcStation) == stationNameToAllNodeIndexes.end() ||
        stationNameToAllNodeIndexes.find(destStation) == stationNameToAllNodeIndexes.end())
    {
        cout << "Invalid station name.\n";
        return;
    }

    vector<int> srcIndices = stationNameToAllNodeIndexes[srcStation];
    vector<int> destIndices = stationNameToAllNodeIndexes[destStation];

    int minDistance = INT_MAX;
    vector<string> bestPath;

    for (int srcIndex : srcIndices)
    {
        vector<int> dist, prev;
        dijkstra(srcIndex, dist, prev);

        for (int destIndex : destIndices)
        {
            if (dist[destIndex] < minDistance)
            {
                minDistance = dist[destIndex];
                bestPath = getPath(srcIndex, destIndex, prev);
            }
        }
    }

    if (bestPath.empty())
    {
        cout << "No path found between stations.\n";
        return;
    }
    cout << "-----------------------------------------------------------------------------------------------------------------------------------\n";
    cout << "                                                                                                                                    \n";
    cout << "                                                                                                                                    \n";
    cout << "Shortest path from '" << srcStation << "' to '" << destStation << "':\n";
    for (size_t i = 0; i < bestPath.size(); ++i)
    {
        cout << bestPath[i];
        if (i < bestPath.size() - 1)
            cout << " -> ";
    }
    cout << "\nDistance: " << minDistance << "\n";
    cout << "                                                                                                                                    \n";
    cout << "                                                                                                                                    \n";
    cout << "-----------------------------------------------------------------------------------------------------------------------------------\n";
}

void display_Stations()
{
    for (int i = 0; i < allStations.size(); i++)
    {
        cout << allStations[i] << endl;
    }
}

void menu()
{
    while (true)
    {
        cout << "\t\t\t\t\n\n~LIST OF ACTIONS~\n\n";
        cout << "1. LIST ALL THE STATIONS IN THE MAP\n";
        cout << "2. TICKET GENERATION\n";
        cout << "3. GET SHORTEST DISTANCE FROM A 'SOURCE' STATION TO 'DESTINATION' STATION\n";
        cout << "4. GROUP JOURNEY PLANNING\n";
        cout << "5. MULTI TRIP PLANNER\n";
        cout << "6. EXIT\n";
        cout << "\nENTER YOUR CHOICE FROM THE ABOVE LIST (1 to 6) : ";

        int choice = -1;
        cin >> choice;

        cout << "\n*\n";

        if (choice == 6)
        {
            break;
        }

        switch (choice)
        {
        case 1:
            display_Stations();
            break;

        case 2:
        {
            string sourceStation;
            while (true)
            {
                cout << "Enter the source station: ";
                cin.ignore();
                getline(cin, sourceStation);

                // Use the search function and check if matches are found
                if (search(sourceStation))
                {
                    break; // Exit the loop if matches are found
                }
                else
                {
                    cout << "No matching stations found. Please try again.\n";
                }
            }

            int a = -1;
            cout << "Select station ID: ";
            cin >> a;

            string destinationStation;
            while (true)
            {
                cout << "Enter the destination station: ";
                cin.ignore();
                cin >> destinationStation;

                // Use the search function and check if matches are found
                if (search(destinationStation))
                {
                    // Exit the loop if matches are found
                    break; 
                }
                else
                {
                    cout << "No matching stations found. Please try again." << endl;
                }
            }

            int b = -1;
            cout << "Select station ID: ";
            cin >> b;

            // Get the shortest path and distance
            vector<int> dist, prev;
            dijkstra(a, dist, prev);
            vector<string> path = getPath(a, b, prev);
            int distance = dist[b];

            // Calculate ticket price (e.g., Rs. 5 per KM)
            double price = distance * 5;
            if (price > 60) {
                price = 60;
            }

            // Generate and print the ticket
            Ticket ticket(allStations[a], allStations[b], path, distance, price);
            ticket.printTicket();
            break;
        }

        case 3:
        {
            string sourceStation;
            while (true)
            {
                cout << "Enter the source station: ";
                cin.ignore();
                getline(cin, sourceStation);

                if (search(sourceStation)) break;
                else {
                    cout << "No matching stations found. Please try again.\n";
                }
            }

            int a = -1;
            cout << "Select station ID: ";
            cin >> a;

            string destinationStation;
            while (true)
            {
                cout << "Enter the destination station: ";
                cin.ignore();
                cin >> destinationStation;

                if (search(destinationStation)) break;
                else {
                    cout << "No matching stations found. Please try again.\n";
                }
            }

            int b = -1;
            cout << "Select station ID: ";
            cin >> b;

            getShortestPath(allStations[a], allStations[b]);
            break;
        }

        case 4:
        {
            cout << "Enter the number of friends: ";
            int numFriends;
            cin >> numFriends;

            vector<int> sources;
            for (int i = 0; i < numFriends; ++i)
            {
                cout << "Enter the starting station for friend " << i + 1 << ": ";
                string station;
                cin.ignore();
                getline(cin, station);
                search(station);
                int stationID;
                cout << "Select station ID: ";
                cin >> stationID;
                sources.push_back(stationID);
            }

            // Find the best meeting point
            int bestStation = findBestMeetingPoint(sources);
            if (bestStation != -1)
            {
                cout << "The best meeting point is: " << allStations[bestStation] << endl;
            }
            else
            {
                cout << "No valid meeting point found.\n";
            }
            break;
        }
        case 5:
        {
            multiTripPlanner();
            break;
        }

        default:
            cout << "Please enter a valid option! " << endl;
            cout << "The options you can choose are from 1 to 6. " << endl;
        }
    }
}

int main()
{
    // Red Line Stations
    vector<string> redLineStations = {
        "Shaheed Sthal (New Bus Adda)", "Hindon River", "Arthala", "Mohan Nagar",
        "Shyam Park", "Major Mohit Sharma Rajendra Nagar", "Raj Bagh", "Shaheed Nagar",
        "Dilshad Garden", "Jhilmil", "Mansarovar Park", "Shahdara", "Welcome",
        "Seelampur", "Shastri Park", "Kashmere Gate", "Tis Hazari", "Pul Bangash",
        "Pratap Nagar", "Shastri Nagar", "Inderlok", "Kanhaiya Nagar", "Keshav Puram",
        "Netaji Subhash Place", "Kohat Enclave", "Pitampura", "Rohini East",
        "Rohini West", "Rithala"};

    // Blue Line Stations (Main Line and Branch Line)
    vector<string> blueLineStations = {
        // Main Line
        "Dwarka Sector 21", "Dwarka Sector 8", "Dwarka Sector 9", "Dwarka Sector 10",
        "Dwarka Sector 11", "Dwarka Sector 12", "Dwarka Sector 13", "Dwarka Sector 14",
        "Dwarka", "Dwarka Mor", "Nawada", "Uttam Nagar West", "Uttam Nagar East",
        "Janakpuri West", "Janakpuri East", "Tilak Nagar", "Subhash Nagar",
        "Tagore Garden", "Rajouri Garden", "Ramesh Nagar", "Moti Nagar",
        "Kirti Nagar", "Shadipur", "Patel Nagar", "Rajendra Place", "Karol Bagh",
        "Jhandewalan", "Ramakrishna Ashram Marg", "Rajiv Chowk", "Barakhamba Road",
        "Mandi House", "Pragati Maidan", "Indraprastha", "Yamuna Bank",
        "Akshardham", "Mayur Vihar Phase-1", "Mayur Vihar Extention",
        "New Ashok Nagar", "Noida Sector 15", "Noida Sector 16", "Noida Sector 18",
        "Botanical Garden", "Golf Course", "Noida City Centre", "Noida Sector 34",
        "Noida Sector 52", "Noida Sector 61", "Noida Sector 59", "Noida Sector 62",
        "Noida Electronic City",
        // Branch Line
        "Yamuna Bank", "Laxmi Nagar", "Nirman Vihar", "Preet Vihar", "Karkarduma",
        "Anand Vihar", "Kaushambi", "Vaishali"};

    // Yellow Line Stations
    vector<string> yellowLineStations = {
        "Samaypur Badli", "Rohini Sector 18,19", "Haiderpur Badli Mor", "Jahangirpuri",
        "Adarsh Nagar", "Azadpur", "Model Town", "GTB Nagar", "Vishwa Vidyalaya",
        "Vidhan Sabha", "Civil Lines", "Kashmere Gate", "Chandni Chowk",
        "Chawri Bazar", "New Delhi", "Rajiv Chowk", "Patel Chowk", "Central Secretariat",
        "Udyog Bhawan", "Lok Kalyan Marg", "Jor Bagh", "INA", "AIIMS", "Green Park",
        "Hauz Khas", "Malviya Nagar", "Saket", "Qutub Minar", "Chhatarpur",
        "Sultanpur", "Ghitorni", "Arjan Garh", "Guru Dronacharya", "Sikandarpur",
        "MG Road", "IFFCO Chowk", "Huda City Centre"};

    // Pink Line Stations
    vector<string> pinkLineStations = {
        "Majlis Park", "Azadpur", "Shalimar Bagh", "Netaji Subhash Place", "Shakurpur",
        "Punjabi Bagh West", "ESI Hospital", "Rajouri Garden", "Mayapuri", "Naraina Vihar",
        "Delhi Cantt", "Durgabai Deshmukh South Campus", "Sir M. Vishweshwaraiah Moti Bagh",
        "Bhikaji Cama Place", "Sarojini Nagar", "INA", "South Extension", "Lajpat Nagar",
        "Vinobapuri", "Ashram", "Hazrat Nizamuddin", "Mayur Vihar Phase-1",
        "Mayur Vihar Pocket-1", "Trilokpuri Sanjay Lake", "Vinod Nagar East",
        "Mandawali - West Vinod Nagar", "IP Extension", "Anand Vihar", "Karkarduma",
        "Karkarduma Court", "Krishna Nagar", "East Azad Nagar", "Welcome", "Jaffrabad",
        "Maujpur - Babarpur", "Gokulpuri", "Johri Enclave", "Shiv Vihar"};

    // Display sample output
    // std::cout << "Total Red Line Stations: " << redLineStations.size() << "\n";
    // std::cout << "Total Blue Line Stations: " << blueLineStations.size() << "\n";
    // std::cout << "Total Yellow Line Stations: " << yellowLineStations.size() << "\n";
    // std::cout << "Total Pink Line Stations: " << pinkLineStations.size() << "\n";

    addNodes(redLineStations, 'R');
    addNodes(blueLineStations, 'B');
    addNodes(yellowLineStations, 'Y');
    addNodes(pinkLineStations, 'P');

    initializeAdjMatrix();

    // Example connections (consecutive stations in each line)
    for (int i = 0; i < redLineStations.size() - 1; ++i) {
        connect(i, i + 1); // R0-R1-R2-R3...
    }

    int offset = redLineStations.size();
    for (int i = 0; i < blueLineStations.size() - 1; ++i) {
        connect(offset + i, offset + i + 1); // B0-B1...
    }

    offset += blueLineStations.size();
    for (int i = 0; i < yellowLineStations.size() - 1; ++i) {
        connect(offset + i, offset + i + 1); // Y0-Y1...
    }

    offset += yellowLineStations.size();
    for (int i = 0; i < pinkLineStations.size() - 1; ++i) {
        connect(offset + i, offset + i + 1); // P0-P1...
    }

    vector<string> interchanges = {
        "Azadpur",
        "Kashmere Gate",
        "Rajiv Chowk",
        "Welcome",
        "Yamuna Bank",
        "Mandi House",
        "Central Secretariat",
        "Anand Vihar",
        "Mayur Vihar Phase-1",
        "INA"};

    for (const auto &s : interchanges) {
        addInterchange(s);
    }

    menu();
    return 0;
}