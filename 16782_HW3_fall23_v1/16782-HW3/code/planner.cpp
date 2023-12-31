#include <iostream>
#include <fstream>
// #include <boost/functional/hash.hpp>
#include <regex>
#include <unordered_set>
#include <set>
#include <list>
#include <unordered_map>
#include <algorithm>
#include <stdexcept>
#include <queue>
#include <climits>
#include <float.h>
#include <math.h>
#include <stack>
#include <string>
#include <vector>



#define SYMBOLS 0
#define INITIAL 1
#define GOAL 2
#define ACTIONS 3
#define ACTION_DEFINITION 4
#define ACTION_PRECONDITION 5
#define ACTION_EFFECT 6

class GroundedCondition;
class Condition;
class GroundedAction;
class Action;
class Env;

using namespace std;

bool print_status = true;

class GroundedCondition
{
private:
    string predicate;
    list<string> arg_values;
    bool truth = true;

public:
    GroundedCondition(string predicate, list<string> arg_values, bool truth = true)
    {
        this->predicate = predicate;
        this->truth = truth;  // fixed
        for (string l : arg_values)
        {
            this->arg_values.push_back(l);
        }
    }

    GroundedCondition(const GroundedCondition& gc)
    {
        this->predicate = gc.predicate;
        this->truth = gc.truth;  // fixed
        for (string l : gc.arg_values)
        {
            this->arg_values.push_back(l);
        }
    }

    string get_predicate() const
    {
        return this->predicate;
    }
    list<string> get_arg_values() const
    {
        return this->arg_values;
    }

    bool get_truth() const
    {
        return this->truth;
    }

    void flip_truth()
    {
        this->truth = !this->truth;
    }

    friend ostream& operator<<(ostream& os, const GroundedCondition& pred)
    {
        os << pred.toString() << " ";
        return os;
    }

    bool operator==(const GroundedCondition& rhs) const
    {
        if (this->predicate != rhs.predicate || this->arg_values.size() != rhs.arg_values.size())
            return false;

        auto lhs_it = this->arg_values.begin();
        auto rhs_it = rhs.arg_values.begin();

        while (lhs_it != this->arg_values.end() && rhs_it != rhs.arg_values.end())
        {
            if (*lhs_it != *rhs_it)
                return false;
            ++lhs_it;
            ++rhs_it;
        }

        if (this->truth != rhs.get_truth()) // fixed
            return false;

        return true;
    }

    string toString() const
    {
        string temp = "";
        temp += this->predicate;
        temp += "(";
        for (string l : this->arg_values)
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

struct GroundedConditionComparator
{
    bool operator()(const GroundedCondition& lhs, const GroundedCondition& rhs) const
    {
        return lhs == rhs;
    }
};

struct GroundedConditionHasher
{
    size_t operator()(const GroundedCondition& gcond) const
    {
        return hash<string>{}(gcond.toString());
    }
};

class Condition
{
private:
    string predicate;
    list<string> args;
    bool truth;

public:
    Condition(string pred, list<string> args, bool truth)
    {
        this->predicate = pred;
        this->truth = truth;
        for (string ar : args)
        {
            this->args.push_back(ar);
        }
    }

    string get_predicate() const
    {
        return this->predicate;
    }

    list<string> get_args() const
    {
        return this->args;
    }

    bool get_truth() const
    {
        return this->truth;
    }

    friend ostream& operator<<(ostream& os, const Condition& cond)
    {
        os << cond.toString() << " ";
        return os;
    }

    bool operator==(const Condition& rhs) const // fixed
    {

        if (this->predicate != rhs.predicate || this->args.size() != rhs.args.size())
            return false;

        auto lhs_it = this->args.begin();
        auto rhs_it = rhs.args.begin();

        while (lhs_it != this->args.end() && rhs_it != rhs.args.end())
        {
            if (*lhs_it != *rhs_it)
                return false;
            ++lhs_it;
            ++rhs_it;
        }

        if (this->truth != rhs.get_truth())
            return false;

        return true;
    }

    string toString() const
    {
        string temp = "";
        if (!this->truth)
            temp += "!";
        temp += this->predicate;
        temp += "(";
        for (string l : this->args)
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

struct ConditionComparator
{
    bool operator()(const Condition& lhs, const Condition& rhs) const
    {
        return lhs == rhs;
    }
};

struct ConditionHasher
{
    size_t operator()(const Condition& cond) const
    {
        return hash<string>{}(cond.toString());
    }
};

class Action
{
private:
    string name;
    list<string> args;
    unordered_set<Condition, ConditionHasher, ConditionComparator> preconditions;
    unordered_set<Condition, ConditionHasher, ConditionComparator> effects;

public:
    Action(string name, list<string> args,
        unordered_set<Condition, ConditionHasher, ConditionComparator>& preconditions,
        unordered_set<Condition, ConditionHasher, ConditionComparator>& effects)
    {
        this->name = name;
        for (string l : args)
        {
            this->args.push_back(l);
        }
        for (Condition pc : preconditions)
        {
            this->preconditions.insert(pc);
        }
        for (Condition pc : effects)
        {
            this->effects.insert(pc);
        }
    }
    string get_name() const
    {
        return this->name;
    }
    list<string> get_args() const
    {
        return this->args;
    }
    unordered_set<Condition, ConditionHasher, ConditionComparator> getPreconditions() const
    {
        return this->preconditions;
    }
    unordered_set<Condition, ConditionHasher, ConditionComparator> getEffects() const
    {
        return this->effects;
    }

    bool operator==(const Action& rhs) const
    {
        if (this->get_name() != rhs.get_name() || this->get_args().size() != rhs.get_args().size())
            return false;

        return true;
    }

    friend ostream& operator<<(ostream& os, const Action& ac)
    {
        os << ac.toString() << endl;
        os << "Precondition: ";
        for (Condition precond : ac.getPreconditions())
            os << precond;
        os << endl;
        os << "Effect: ";
        for (Condition effect : ac.getEffects())
            os << effect;
        os << endl;
        return os;
    }

    string toString() const
    {
        string temp = "";
        temp += this->get_name();
        temp += "(";
        for (string l : this->get_args())
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

struct ActionComparator
{
    bool operator()(const Action& lhs, const Action& rhs) const
    {
        return lhs == rhs;
    }
};

struct ActionHasher
{
    size_t operator()(const Action& ac) const
    {
        return hash<string>{}(ac.get_name());
    }
};

class Env
{
private:
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> initial_conditions;
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> goal_conditions;
    unordered_set<Action, ActionHasher, ActionComparator> actions;
    unordered_set<string> symbols;

public:
    // Added ----------------
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> get_initial_condition()
    {
        return this->initial_conditions;
    }
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> get_goal_condition()
    {
        return this->goal_conditions;
    }
    // ----------------------
    void remove_initial_condition(GroundedCondition gc)
    {
        this->initial_conditions.erase(gc);
    }
    void add_initial_condition(GroundedCondition gc)
    {
        this->initial_conditions.insert(gc);
    }
    void add_goal_condition(GroundedCondition gc)
    {
        this->goal_conditions.insert(gc);
    }
    void remove_goal_condition(GroundedCondition gc)
    {
        this->goal_conditions.erase(gc);
    }
    void add_symbol(string symbol)
    {
        symbols.insert(symbol);
    }
    void add_symbols(list<string> symbols)
    {
        for (string l : symbols)
            this->symbols.insert(l);
    }
    void add_action(Action action)
    {
        this->actions.insert(action);
    }

    Action get_action(string name)
    {
        for (Action a : this->actions)
        {
            if (a.get_name() == name)
                return a;
        }
        throw runtime_error("Action " + name + " not found!");
    }

    unordered_set<Action, ActionHasher, ActionComparator> get_actions()
    {
        return this->actions;
    }

    unordered_set<string> get_symbols() const
    {
        return this->symbols;
    }

    friend ostream& operator<<(ostream& os, const Env& w)
    {
        os << "***** Environment *****" << endl << endl;
        os << "Symbols: ";
        for (string s : w.get_symbols())
            os << s + ",";
        os << endl;
        os << "Initial conditions: ";
        for (GroundedCondition s : w.initial_conditions)
            os << s;
        os << endl;
        os << "Goal conditions: ";
        for (GroundedCondition g : w.goal_conditions)
            os << g;
        os << endl;
        os << "Actions:" << endl;
        for (Action g : w.actions)
            os << g << endl;
        cout << "***** Environment Created! *****" << endl;
        return os;
    }
};

class GroundedAction
{
private:
    string name;
    list<string> arg_values;
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> gPreconditions;
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> gEffects;

public:
    GroundedAction(string name, list<string> arg_values)
    {
        this->name = name;
        for (string ar : arg_values)
        {
            this->arg_values.push_back(ar);
        }
    }

    GroundedAction(string name, list<string> arg_values,
                   unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> g_precon,
                   unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> g_effect)
    {
        this->name = name;
        for (string ar : arg_values)
            this->arg_values.push_back(ar);
        for (GroundedCondition gc : g_precon)
            this->gPreconditions.insert(gc);
        for (GroundedCondition gc : g_effect)
            this->gEffects.insert(gc);
    }
    
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> getPreconditions()
    {
        return this->gPreconditions;
    }

    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> getEffects()
    {
        return this->gEffects;
    }
    string get_name() const
    {
        return this->name;
    }

    list<string> get_arg_values() const
    {
        return this->arg_values;
    }

    bool operator==(const GroundedAction& rhs) const
    {
        if (this->name != rhs.name || this->arg_values.size() != rhs.arg_values.size())
            return false;

        auto lhs_it = this->arg_values.begin();
        auto rhs_it = rhs.arg_values.begin();

        while (lhs_it != this->arg_values.end() && rhs_it != rhs.arg_values.end())
        {
            if (*lhs_it != *rhs_it)
                return false;
            ++lhs_it;
            ++rhs_it;
        }
        return true;
    }

    friend ostream& operator<<(ostream& os, const GroundedAction& gac)
    {
        os << gac.toString() << " ";
        return os;
    }

    string toString() const
    {
        string temp = "";
        temp += this->name;
        temp += "(";
        for (string l : this->arg_values)
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

list<string> parse_symbols(string symbols_str)
{
    list<string> symbols;
    size_t pos = 0;
    string delimiter = ",";
    while ((pos = symbols_str.find(delimiter)) != string::npos)
    {
        string symbol = symbols_str.substr(0, pos);
        symbols_str.erase(0, pos + delimiter.length());
        symbols.push_back(symbol);
    }
    symbols.push_back(symbols_str);
    return symbols;
}

Env* create_env(char* filename)
{
    ifstream input_file(filename);
    Env* env = new Env();
    regex symbolStateRegex("symbols:", regex::icase);
    regex symbolRegex("([a-zA-Z0-9_, ]+) *");
    regex initialConditionRegex("initialconditions:(.*)", regex::icase);
    regex conditionRegex("(!?[A-Z][a-zA-Z_]*) *\\( *([a-zA-Z0-9_, ]+) *\\)");
    regex goalConditionRegex("goalconditions:(.*)", regex::icase);
    regex actionRegex("actions:", regex::icase);
    regex precondRegex("preconditions:(.*)", regex::icase);
    regex effectRegex("effects:(.*)", regex::icase);
    int parser = SYMBOLS;

    unordered_set<Condition, ConditionHasher, ConditionComparator> preconditions;
    unordered_set<Condition, ConditionHasher, ConditionComparator> effects;
    string action_name;
    string action_args;

    string line;
    if (input_file.is_open())
    {
        while (getline(input_file, line))
        {
            string::iterator end_pos = remove(line.begin(), line.end(), ' ');
            line.erase(end_pos, line.end());

            if (line == "")
                continue;

            if (parser == SYMBOLS)
            {
                smatch results;
                if (regex_search(line, results, symbolStateRegex))
                {
                    line = line.substr(8);
                    sregex_token_iterator iter(line.begin(), line.end(), symbolRegex, 0);
                    sregex_token_iterator end;

                    env->add_symbols(parse_symbols(iter->str()));  // fixed

                    parser = INITIAL;
                }
                else
                {
                    cout << "Symbols are not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == INITIAL)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, initialConditionRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        if (predicate[0] == '!')
                        {
                            env->remove_initial_condition(
                                GroundedCondition(predicate.substr(1), parse_symbols(args)));
                        }
                        else
                        {
                            env->add_initial_condition(
                                GroundedCondition(predicate, parse_symbols(args)));
                        }
                    }

                    parser = GOAL;
                }
                else
                {
                    cout << "Initial conditions not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == GOAL)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, goalConditionRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        if (predicate[0] == '!')
                        {
                            env->remove_goal_condition(
                                GroundedCondition(predicate.substr(1), parse_symbols(args)));
                        }
                        else
                        {
                            env->add_goal_condition(
                                GroundedCondition(predicate, parse_symbols(args)));
                        }
                    }

                    parser = ACTIONS;
                }
                else
                {
                    cout << "Goal conditions not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTIONS)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, actionRegex))
                {
                    parser = ACTION_DEFINITION;
                }
                else
                {
                    cout << "Actions not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTION_DEFINITION)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, conditionRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;
                    // name
                    action_name = iter->str();
                    iter++;
                    // args
                    action_args = iter->str();
                    iter++;

                    parser = ACTION_PRECONDITION;
                }
                else
                {
                    cout << "Action not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTION_PRECONDITION)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, precondRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        bool truth;

                        if (predicate[0] == '!')
                        {
                            predicate = predicate.substr(1);
                            truth = false;
                        }
                        else
                        {
                            truth = true;
                        }

                        Condition precond(predicate, parse_symbols(args), truth);
                        preconditions.insert(precond);
                    }

                    parser = ACTION_EFFECT;
                }
                else
                {
                    cout << "Precondition not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTION_EFFECT)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, effectRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        bool truth;

                        if (predicate[0] == '!')
                        {
                            predicate = predicate.substr(1);
                            truth = false;
                        }
                        else
                        {
                            truth = true;
                        }

                        Condition effect(predicate, parse_symbols(args), truth);
                        effects.insert(effect);
                    }

                    env->add_action(
                        Action(action_name, parse_symbols(action_args), preconditions, effects));

                    preconditions.clear();
                    effects.clear();
                    parser = ACTION_DEFINITION;
                }
                else
                {
                    cout << "Effects not specified correctly." << endl;
                    throw;
                }
            }
        }
        input_file.close();
    }

    else
        cout << "Unable to open file";

    return env;
}

struct node
{
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> state;

    double f = std::numeric_limits<double>::infinity();
    double g = std::numeric_limits<double>::infinity();
    double h = std::numeric_limits<double>::infinity();
    
    int parentIndex = -1;

    string parentNodeState = "";
};

class Planner
{ 
public:  
    Env* env;                                               // Environment
    stack<GroundedAction> path;                             // Path from initial to goal state
    int whichHeur = 1;                                      // Heuristic Type
    vector<vector<string>> combinations, permutations;      // Combinations and Permutations
    vector<string> comb;                                    // Temporary Combination
    vector<GroundedAction> allGroundedActions;                          // All Grounded Actions
    unordered_set<Action, ActionHasher, ActionComparator> allActions;   // All Actions
    vector<string> allSymbols;                                          // All Symbols

    // Constructor
    Planner(Env* env)
    {
        this->env = env; 
    }

    // Get Grounded Actions
    void getAllGroundedActions(Action &action, vector<vector<string>> &args)
    {
        //Store the actions preconditions and effects
        unordered_set<Condition, ConditionHasher, ConditionComparator> preconditions = action.getPreconditions();
        unordered_set<Condition, ConditionHasher, ConditionComparator> effects = action.getEffects();

        // Iterate over each permutation of arguments
        for(vector<string> arg : args)
        {
            list<string> gaArgs(arg.begin(), arg.end());  // The specific set of symbols being used
            
            unordered_map<string, string> argMap;   // Maps the general action symbols to the corresponding grounded action symbols
            list<string> action_args = action.get_args();

            // Building the Map
            list<string>::const_iterator action_it, gaction_it;
            for (action_it = action_args.begin(), gaction_it = gaArgs.begin(); action_it != action_args.end(); ++action_it, ++gaction_it)
                argMap[*action_it] = *gaction_it;
            
            unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> groundedPreconditions;
            unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> groundedEffects;

            // Grounding the preconditions
            for(Condition precondition : preconditions)
            {   
                list<string> gcArgs;
                list<string> precondArgs = precondition.get_args();  // Make direct
                list<string>::const_iterator precond_it;
                for(precond_it = precondArgs.begin(); precond_it != precondArgs.end() ; ++precond_it)
                {
                    if(argMap[*precond_it] == "")
                        gcArgs.push_back(*precond_it);
                    else
                        gcArgs.push_back(argMap[*precond_it]);
                }
                
                GroundedCondition gc(precondition.get_predicate(), gcArgs, precondition.get_truth());
                groundedPreconditions.insert(gc); 
            }
            
            // Grounding the effects
            for(Condition effect : effects)
            {
                list<string> gcArgs;
                list<string> effectArgs = effect.get_args();  // Make direct
                list<string>::const_iterator effect_it;
                for(effect_it = effectArgs.begin(); effect_it != effectArgs.end() ; ++effect_it)
                {
                    if(argMap[*effect_it] == "")
                        gcArgs.push_back(*effect_it);
                    else
                        gcArgs.push_back(argMap[*effect_it]);
                }

                GroundedCondition gc(effect.get_predicate(), gcArgs, effect.get_truth());
                groundedEffects.insert(gc);
            }
            GroundedAction ga(action.get_name(), gaArgs, groundedPreconditions, groundedEffects);
            allGroundedActions.push_back(ga);
        }
    }

    // Check if Goal is reached
    bool goalReached(node & currNode)
    {
        for(GroundedCondition gc : env->get_goal_condition()) 
            if(currNode.state.find(gc) == currNode.state.end())
                return 0;
        return 1;
    }

    // Backtrack to get the path
    void getPath(string& goalNodeStr, unordered_map<string, node>& nodeInfo, string nodeStr)
    {
        string currNodeStr = goalNodeStr;
        while(currNodeStr != nodeStr)
        {
            path.push(allGroundedActions[nodeInfo[currNodeStr].parentIndex]);
            currNodeStr = nodeInfo[currNodeStr].parentNodeState;
        }
        return;
    }

    // Hash the state
    string symbolichash(unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator>& stateset)
    {
        set<string> stringState;
        string hashedString = "";

        // GroundedCondition in the stateset into a string and storing these strings in the stringState set
        for(GroundedCondition gc : stateset)
            stringState.insert(gc.toString());

        // Concatenating the strings in the stringState set to get a single string
        for (auto it = stringState.begin(); it != stringState.end(); it++) 
            hashedString += *it; 
        
        return hashedString;
    }

    // Get the path length
    double pathLength(string& goalNodeStr, unordered_map<string, node>& nodeInfo, string& h_initNodeStr)
    {
        string currNodeStr = goalNodeStr;
        double pathlength = 0;
        while(currNodeStr != h_initNodeStr)
        {
            ++pathlength;
            currNodeStr = nodeInfo[currNodeStr].parentNodeState;
        }
        return pathlength;
    }

    // Empty-Delete-List Heuristic
    double edl_calc(unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator>& heurNode)
    {
        unordered_map<string, bool> closedList;     //closedList of bool values for each cell
        unordered_map<string, node> nodeInfo;
        priority_queue<pair<double, string>, vector<pair<double, string>>, greater<pair<double, string>>> openList;   //f-value, node index (sorted in increasing order of f-value)

        node startNode;
        startNode.state = heurNode;
        string h_initNodeStr = symbolichash(startNode.state);
        startNode.g = 0;
        startNode.f = 0;
        startNode.h = 0;
        nodeInfo[h_initNodeStr] = startNode;
        openList.push(make_pair(startNode.f, h_initNodeStr));

        while (!openList.empty())
        {
            pair<double, string> curNodeStr = openList.top();
            openList.pop();

            // Check if current state has been expanded before
            if(closedList[curNodeStr.second] == true)
                continue;

            closedList[curNodeStr.second] = true;
            node curNode = nodeInfo[curNodeStr.second];

            // Check if current state satisfies the Goal Conditions
            if(goalReached(curNode)) 
                return pathLength(curNodeStr.second, nodeInfo, h_initNodeStr);
             
            bool ActionBool = 1;
            int counter = -1;

            string hashedNode;

            // Find all the Valid Grounded Actions from this state
            for(GroundedAction ga : this->allGroundedActions)
            {
                counter++;
                ActionBool = 1;

                //Find out if the action can be taken
                for(GroundedCondition gc : ga.getPreconditions())
                {   
                    if(curNode.state.find(gc) == curNode.state.end())
                    {
                        ActionBool = 0;
                        break;
                    }
                }

                if(ActionBool)
                {
                    node newNode;

                    // Apply the action
                    newNode.state = curNode.state;
                    
                    // Insert only the positive effects and ignore any deletions
                    for(GroundedCondition eff : ga.getEffects())
                    {
                        // printf("Effect: %s\n", eff.toString().c_str());
                        if(eff.get_truth())
                            newNode.state.insert(eff);
                    }

                    // printf("New Node: %s\n", symbolichash(newNode.state).c_str());
                    hashedNode = symbolichash(newNode.state);
                    if (closedList[hashedNode])
                        continue;

                    newNode.g = curNode.g + 1;

                    double heur = 0;
                    for(GroundedCondition hgc : env->get_goal_condition())  
                        if(newNode.state.find(hgc) == newNode.state.end())
                            ++heur;
            
                    newNode.h = heur;
                    newNode.f = newNode.g + newNode.h;

                    if(nodeInfo.find(hashedNode) == nodeInfo.end() || newNode.g < nodeInfo[hashedNode].g)
                    {
                        newNode.parentIndex = counter;
                        newNode.parentNodeState = curNodeStr.second;
                        nodeInfo[hashedNode] = newNode;
                        openList.push(make_pair(newNode.f, hashedNode));
                    }    
                }             
            }
        }
        return 0;
    }

    double getHeuristic(unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator>& curNode)
    {
        // Heuristic 1: Set Heuristic to number of goal conditions not satisfied
        if (whichHeur == 1)
        {
            double heur = 0;
            for(GroundedCondition gc : env->get_goal_condition())
            {   
                if(curNode.find(gc) == curNode.end())
                    ++heur;
            }
            return heur;
        }

        // Heuristic 2: Set Heuristic to Empty-Delete-List
        else if(whichHeur == 2)
            return edl_calc(curNode);

        // Heuristic 3: Set Heuristic to 0
        else if(whichHeur == 3)
            return 0;

        return 0;
    }

    // Get all Combinations
    void getCombinations(int offset, int k, int numOfSym)
    {
        if (k == 0) 
        {
            combinations.push_back(comb);
            return;
        }
        for (int i = offset; i <= numOfSym - k; ++i) 
        {
            comb.push_back(allSymbols[i]);
            getCombinations(i+1, k-1, numOfSym);
            comb.pop_back();
        }
    }

    // Get all Permutations
    void getPermutations(vector<string> &comb)
    {
        sort(comb.begin(), comb.end()); 
        do
        { 
            permutations.push_back(comb); 
        } while (next_permutation(comb.begin(), comb.end()));
    }

    // Preprocessing of the environment : Get all Grounded Actions
    void preprocessing()
    {
        // Store all Actions
        allActions = env->get_actions();

        // Store all Symbols as a Vetor
        vector<string> allSymbolsVect(env->get_symbols().begin(), env->get_symbols().end());
        allSymbols = allSymbolsVect;
        int numOfSym = allSymbols.size();

        // Build the list of all Grounded Actions
        for(Action action : allActions)
        {
            int numOfArgs = action.get_args().size();
            getCombinations(0, numOfArgs, numOfSym);
            for(int i=0 ; i<combinations.size() ; i++)
                getPermutations(combinations[i]);
            
            // Send permutations to actions to get grounded actions
            getAllGroundedActions(action, permutations);

            combinations.clear();
            comb.clear();
            permutations.clear();
        }
    }

    void Astar()
    {
        cout<<"Starting Astar\n"<<endl;

        unordered_map<string, bool> closedList;     //closedList of bool values for each cell
        unordered_map<string, node> nodeInfo;       //nodeInfo of node for each cell
        priority_queue<pair<double, string>, vector<pair<double, string>>, greater<pair<double, string>>> openList;   //f-value, node index (sorted in increasing order of f-value)

        // Initialize the initial node
        node startNode;
        startNode.state = this->env->get_initial_condition();
        string nodeStr = symbolichash(startNode.state);
        startNode.g = 0;
        startNode.f = 0;
        nodeInfo[nodeStr] = startNode;
        openList.push(make_pair(startNode.f, symbolichash(startNode.state)));

        int numStates = 0;

        while (!openList.empty())
        {
            printf("Length of Open List = %ld | Number of nodes expanded : %ld \n", openList.size(), nodeInfo.size());
            pair<double, string> curNodeStr = openList.top();
            openList.pop();

            // Check if current state has been expanded before
            if(closedList[curNodeStr.second] == true)
                continue;
            closedList[curNodeStr.second] = true;
            numStates++;

            node curNode = nodeInfo[curNodeStr.second];

            // Check if current state satisfies the Goal Conditions
            if(goalReached(curNode)) 
            {
                // Print the number of expanded nodes
                printf("The number of expanded Nodes : %ld\n", nodeInfo.size());

                getPath(curNodeStr.second, nodeInfo, nodeStr);
                return;
            }
                
            bool ActionBool = 1;
            int counter = -1;

            string hashedNode;

            for(GroundedAction ga : this->allGroundedActions)
            {
                counter++;
                ActionBool = 1;

                //Find out if the action can be taken
                for(GroundedCondition gc : ga.getPreconditions())
                {   
                    if(curNode.state.find(gc) == curNode.state.end())
                    {
                        ActionBool = 0;
                        break;
                    }
                }

                if(ActionBool)
                {
                    node newNode;

                    // Apply the action
                    newNode.state = curNode.state;
                    for(GroundedCondition eff : ga.getEffects())
                    {
                        if(eff.get_truth())
                            newNode.state.insert(eff);
                        else
                        {
                            // Remove from state
                            eff.flip_truth();
                            newNode.state.erase(newNode.state.find(eff));
                        }
                    }

                    hashedNode = symbolichash(newNode.state);
                    if (closedList[hashedNode])
                        continue;

                    newNode.g = curNode.g + 1;
                    newNode.h = getHeuristic(newNode.state);
                    newNode.f = newNode.g + newNode.h;

                    // If the node has not been seen before (or) if the node now has a lesser cost path
                    if(nodeInfo.find(hashedNode) == nodeInfo.end() || newNode.g < nodeInfo[hashedNode].g)
                    {
                        newNode.parentIndex = counter;
                        newNode.parentNodeState = curNodeStr.second;
                        nodeInfo[hashedNode] = newNode;
                        openList.push(make_pair(newNode.f, hashedNode));
                    }    
                }
                
            }
        }
    }

};

list<GroundedAction> planner(Env* env)
{
    list<GroundedAction> actions;
    
    clock_t t;
    t = clock();
    Planner p(env);

    // Set Which Heuristic to use
    // 1 - Number of Goal Conditions not satisfied
    // 2 - Empty-Delete-List
    // 3 - Default to 0

    p.whichHeur = 3;
    p.preprocessing();
    p.Astar();
    
    t = clock() - t;
    cout<<"Time Taken: "<<((float)t)/CLOCKS_PER_SEC<<" seconds\n";

    while(!p.path.empty())
    {
        actions.push_back(p.path.top());
        p.path.pop();
    }

    return actions;
}

int main(int argc, char* argv[])
{
    // DO NOT CHANGE THIS FUNCTION
    char* filename = (char*)("example.txt");
    if (argc > 1)
        filename = argv[1];

    cout << "Environment: " << filename << endl << endl;
    Env* env = create_env(filename);
    if (print_status)
    {
        cout << *env;
    }

    list<GroundedAction> actions = planner(env);

    cout << "\nPlan: " << endl;
    for (GroundedAction gac : actions)
    {
        cout << gac << endl;
    }

    return 0;
}