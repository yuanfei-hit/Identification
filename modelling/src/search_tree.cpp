#pragma once
#include "head.h"
#include "model_builder.h"
#include "model.h"


// class HeightNode
// {
// public:
//     pair<float, float> lower;
//     pair<float, float> thumb;
//     pair<float, float> upper;
//     vector<pair<float, float> > subset;
//     vector<HeightNode*> children;
//     HeightNode* parent;
//     HeightNode(vector<pair<float, float> > subset)
//     {
//         this->lower = pair<float, float>(-1, -1);
//         this->thumb = pair<float, float>(-1, -1);
//         this->upper = pair<float, float>(-1, -1);
//         this->parent = NULL;
//         this->subset = subset;
//     };
//     HeightNode(vector<pair<float, float> > data, vector<pair<float, float> > subset)
//     {
//         if (data.size() == 1)
//         {
//             this->lower = pair<float, float>(-1, -1);
//             this->thumb = data[0];
//             this->upper = pair<float, float>(-1, -1);
//         }
//         else
//         {
//             if (data.size() == 3)
//             {
//                 this->lower = data[0];
//                 this->thumb = data[1];
//                 this->upper = data[2];
//             }
//         }
//         this->parent = NULL;
//         this->subset = subset;
//     };
//     bool isLeaf()
//     {
//         if (this->subset.size() == 0)
//             return true;
//         return false;
//     };
// };

// class HeightSearchTree
// {
// public:
//     vector<float> heights;
//     HeightNode* head;
//     HeightSearchTree(vector<pair<float, float> > data);
//     vector<vector<pair<float, float> > > separate(vector<pair<float, float> > data);
//     vector<HeightNode*> findSolution(vector<pair<float, float> > data);
//     bool isSuitable(pair<float, float> lower, pair<float, float> thumb, pair<float, float> upper);
//     bool isInSpan(pair<float, float> span, float height);

// };
// HeightSearchTree::HeightSearchTree(vector<pair<float, float> > data)
// {
//     this->head = new HeightNode(data);
//     // printf("head size: %d \n", this->head->subset.size());
// }
// vector<vector<pair<float, float> > > HeightSearchTree::separate(vector<pair<float, float> > data)
// {
//     vector<vector<pair<float, float> > > res;
//     for (int i = 0; i < data.size(); i ++)
//     {
//         vector<pair<float, float> > temp;
//         temp.push_back(data[i]);
//         res.push_back(temp);
//     }
//     return res;
// }
// vector<HeightNode*> HeightSearchTree::findSolution(vector<pair<float, float> > data)
// {
//     vector<HeightNode*> res;
//     vector<pair<float, float> > empty_vector;
//     if (data.size() < 3)
//     {
//         vector<vector<pair<float, float> > > separated = separate(data);
//         for (int i = 0; i < separated.size(); i++)
//         {
//             HeightNode* node = new HeightNode(separated[i], empty_vector);
//             res.push_back(node);
//         }
//     }
//     else
//     {
//         bool found = false;
//         for(int i = 0; i <data.size(); i ++)
//         {
//             for (int j = i + 1; j < data.size(); j ++)
//             {
//                 for (int k = j + 1; k < data.size(); k ++)
//                 {
//                     bool suit = isSuitable(data[i], data[j], data[k]);
//                     if (suit)
//                     {
//                         found = true;
//                         vector<pair<float, float> > temp;
//                         temp.push_back(data[i]);
//                         temp.push_back(data[j]);
//                         temp.push_back(data[k]);
//                         vector<pair<float, float> > data_copy;
//                         for(int m = 0; m < data.size(); m ++)
//                             if (m != i && m != j && m != k)
//                                 data_copy.push_back(data[m]);
//                         HeightNode* node = new HeightNode(temp, data_copy);
//                         res.push_back(node);
//                     }
//                 }
//             }
//         }
//         if (!found)
//         {
//             vector<vector<pair<float, float> > > separated = separate(data);
//             for (int i = 0; i < separated.size(); i++)
//             {
//                 HeightNode* node = new HeightNode(separated[i], empty_vector);
//                 res.push_back(node);
//             }
//         }
//     }
//     return res;
// }

// bool HeightSearchTree::isInSpan(pair<float, float> span, float height)
// {
//     if (height < span.second && height >= span.first)
//         return true;
//     return false;
// }
// bool HeightSearchTree::isSuitable(pair<float, float> lower, pair<float, float> thumb, pair<float, float> upper)
// {
//     float step = 0.001;
//     set<float> heights;
//     float dist = 0.033;
//     float save_distance = 0.03;
//     float height;
//     for (float i = thumb.first; i < thumb.second; i += step)
//     {
//         float lower_bound = i - dist;
//         float upper_bound = i + dist;
//         if (lower_bound + save_distance <= 0)
//         {

//         }
//         else
//         {
//             if (isInSpan(lower, lower_bound) && isInSpan(upper, upper_bound))
//                 heights.insert(i);
//         }
//     }
//     set<float>::iterator it;
//     vector<float> height_vector;
//     for (it = heights.begin(); it != heights.end(); ++it)
//     {
//         height_vector.push_back(*it);
//     }
//     if (height_vector.size() != 0)
//     {
//         height = height_vector[(int)(height_vector.size()/2)];
//         return true;
//     }
//     else
//     {
//         height = -1;
//         return false;
//     }
// }
// vector<pair<float, float> > reSchedule(vector<pair<float, float> > input)
// {
//     set<float> height_set;
//     for (int i = 0; i < input.size(); i ++)
//     {
//         height_set.insert(input[i].first);
//         height_set.insert(input[i].second);
//     }
//     set<float>::iterator it;
//     vector<float> heights;
//     vector<pair<float, float> > res;
//     for (it = height_set.begin(); it != height_set.end(); ++ it)
//     {
//         heights.push_back(*it);
//     }
//     for (int i = 0; i < heights.size() - 1; i++)
//     {
//         res.push_back(pair<float, float>(heights[i], heights[i+1]));
//     }
//     return res;
// }

int main(int argc, char** argv)
{

    // vector<pair<float, float> > heights;
    // heights.push_back(pair<float, float>(0.03, 0.036));
    // heights.push_back(pair<float, float>(0.051, 0.066));
    // heights.push_back(pair<float, float>(0.036, 0.195));
    // heights.push_back(pair<float, float>(0.051, 0.195));
    // heights.push_back(pair<float, float>(0.054, 0.195));
    // heights.push_back(pair<float, float>(0.135, 0.156));

    // vector<pair<float, float> > reschedule = reSchedule(heights);
    // stack<HeightNode*> st;
    // HeightSearchTree tree = HeightSearchTree(reschedule);
    // st.push(tree.head);
    // while(!st.empty())
    // {
    //     HeightNode* node = st.top();
    //     st.pop();
    //     if (node->isLeaf() == true)
    //     {
    //         continue;
    //     }
    //     vector<pair<float, float> > vector_temp = node->subset;
    //     vector<HeightNode*> res = tree.findSolution(vector_temp);
    //     node->children = res;
    //     for (int i = 0; i < res.size(); i ++)
    //     {
    //         res[i]->parent = node;
    //         st.push(res[i]);
    //     }
    // }

    // queue<HeightNode*> q;
    // q.push(tree.head);
    // vector<pair<float, float> > result;
    // while(!q.empty())
    // {
    //     HeightNode* queue_point = q.front();
    //     if (queue_point->isLeaf())
    //     {
    //         HeightNode* temp = queue_point->parent;
    //         for (int i = 0; i < temp->children.size(); i ++)
    //         {
    //             result.push_back(temp->children[i]->thumb);
    //         }
    //         HeightNode* search_temp = temp;
    //         while(search_temp->parent != NULL)
    //         {
    //             result.push_back(search_temp->thumb);
    //             search_temp = search_temp->parent;
    //         }
    //         break;
    //     }
    //     q.pop();
    //     vector<HeightNode*> temp = queue_point->children;
    //     for (int i = 0; i < temp.size(); i ++)
    //     {
    //         q.push(temp[i]);
    //     }
    // }

    // printf("----------------------------\n");
    // for (int i = 0; i < result.size(); i++)
    // {
    //     printf("[ %f ~ %f ] \n", result[i].first, result[i].second);
    // }














    if (string(argv[1]) == "build")
    {
        vector<string> filenames;
        for (int i = 2; i < argc; i++)
        {
            printf("Load models: %s \n", argv[i]);
            filenames.push_back(string(argv[i]));
        }
        ModelBuilder builder = ModelBuilder(filenames);
    }
    else if (string(argv[1]) == "read")
    {
        Database database;
        database.readDatabase("database");
        database.info();
        stringstream ss;
        int id;
        ss << argv[2] ;
        ss >> id;
        int index = database.search(id);
        database.setHandParameter(0.033, 0.03);
        vector<pair<float, float> > height = database.getHeight(id);
        printf("[ id ]: %d \n", id);
        printf("[ index ]: %d \n", index);
        printf("[ height ]: ");
        for (int i = 0; i < height.size(); i++)
        {
            printf("%f - %f ", height[i].first, height[i].second);
        }
        printf("\n");
    }

  
    return (0);
}