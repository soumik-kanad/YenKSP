// C program for Dijkstra's single 
// source shortest path algorithm.
// The program is for adjacency matrix
// representation of the graph.
#include <stdio.h>
#include <limits.h>
#include <stdlib.h>
 
// Number of vertices 
// in the graph
#define MAX 100
#define MAX_CONTAINER_SIZE 10000
#define V 9


//defining boolean , because it is not present in C
enum bool{false, true};
typedef enum bool bool;

int not(bool * b)
{
  if(*b==true)
    return false;
  return true;
}


typedef struct path{
  int pathlist[MAX];
  int pathlen;
} path;

//initializes pathlist by -1 and path lenth by INTMAX
void initialise(path *P)
{
  int i;
  for(i=0;i<MAX;i++)
    P->pathlist[i]=-1;
  P->pathlen=INT_MAX;
}

//Returns lenth of a sub-path given a pathlist
int pathLenth(int graph[V][V],int pathlist[], int startidx, int endidx)
{
  int len=0;
  for(int i=startidx;i<endidx;i++)
    len+=graph[pathlist[i]][pathlist[i+1]];
  return len;
}

//Returns a subpath of path P between two given indices
path subPath(int graph[V][V], path *P, int startidx,int endidx)
{
  path q;
  initialise(&q);
  for(int i=0;i<endidx-startidx+1;i++)
    q.pathlist[i]=P->pathlist[i+startidx];
  q.pathlen=pathLenth(graph,q.pathlist,0,endidx-startidx);
  return q;
}

//Prints paths and pth length
void printPath(path P)
{
  int no_of_nodes=0;
  while(P.pathlist[no_of_nodes]!=-1)
    no_of_nodes++;

  if(no_of_nodes<2)
  {
    printf("incomplete path");
    return;
  }

  printf("Path: %d",P.pathlist[0]);
  for(int i=1;i<no_of_nodes;i++)
    printf("-%d",P.pathlist[i] );
  printf("\nLength:%d\n",P.pathlen);

}



 
// A utility function to find the 
// vertex with minimum distance
// value, from the set of vertices
// not yet included in shortest
// path tree
int minDistance(int dist[], 
                bool sptSet[])
{
     
    // Initialize min value
    int min = INT_MAX, min_index;
 
    for (int v = 0; v < V; v++)
        if (sptSet[v] == false &&
                   dist[v] <= min)
            min = dist[v], min_index = v;
 
    return min_index;
}
 
// // Function to print shortest
// // path from source to j
// // using parent array
// void printPath(int parent[], int j)
// {
     
//     // Base Case : If j is source
//     if (parent[j] == - 1)
//         return;
 
//     printPath(parent, parent[j]);
 
//     printf("%d ", j);
// }
 
// // A utility function to print 
// // the constructed distance
// // array
// int printSolution(int dist[], int n, 
//                       int parent[])
// {
//     int src = 0;
//     printf("Vertex\t Distance\tPath");
//     for (int i = 1; i < V; i++)
//     {
//         printf("\n%d -> %d \t\t %d\t\t%d ",
//                       src, i, dist[i], src);
//         printPath(parent, i);
//     }
// }

void getPath(int parent[], int j, int count, int revPathList[])
{
  if(parent[j] == -1)
    return;
  
  getPath(parent, parent[j], count+1, revPathList);
  

  //printf("[%d] = %d", count, j);
  revPathList[count]=j;

}

path getShortestPath(int src, int dest, int dist[], int parent[])
{
  path P;
  initialise(&P);

  int count=0;
  int revPathList[MAX];
  for(int i=0;i<MAX;i++)
    revPathList[i]=-1;
  

  getPath(parent, dest, count, revPathList);

  int n=0;
  while(revPathList[n]!=-1)
    n++;

  P.pathlist[0]= src;
  for(int i=1;i<=n;i++)
    P.pathlist[i]=revPathList[n-i];

  P.pathlen=dist[dest];

  return P;
}
 
// Funtion that implements Dijkstra's
// single source shortest path
// algorithm for a graph represented
// using adjacency matrix representation
path dijkstra(int graph[V][V], int src, int dest)
{
     
    // The output array. dist[i]
    // will hold the shortest
    // distance from src to i
    int dist[V]; 
 
    // sptSet[i] will true if vertex
    // i is included / in shortest
    // path tree or shortest distance 
    // from src to i is finalized
    bool sptSet[V];
 
    // Parent array to store
    // shortest path tree
    int parent[V];
 
    // Initialize all distances as 
    // INFINITE and stpSet[] as false
    for (int i = 0; i < V; i++)
    {
        parent[0] = -1;
        dist[i] = INT_MAX;
        sptSet[i] = false;
    }
 
    // Distance of source vertex 
    // from itself is always 0
    dist[src] = 0;
 
    // Find shortest path
    // for all vertices
    for (int count = 0; count < V - 1; count++)
    {
        // Pick the minimum distance
        // vertex from the set of
        // vertices not yet processed. 
        // u is always equal to src
        // in first iteration.
        int u = minDistance(dist, sptSet);
 
        // Mark the picked vertex 
        // as processed
        sptSet[u] = true;
 
        // Update dist value of the 
        // adjacent vertices of the
        // picked vertex.
        for (int v = 0; v < V; v++)
 
            // Update dist[v] only if is
            // not in sptSet, there is
            // an edge from u to v, and 
            // total weight of path from
            // src to v through u is smaller
            // than current value of
            // dist[v]
            if (not(&sptSet[v]) && graph[u][v] &&
                dist[u] + graph[u][v] < dist[v])
            {
                parent[v] = u;
                dist[v] = dist[u] + graph[u][v];
            } 
    }
 
    // print the constructed
    // distance array
    
    //printSolution(dist, V, parent);
    return getShortestPath(src, dest, dist, parent);

}

int comparator(const void *p, const void *q) 
{
    int l = ((struct path *)p)->pathlen;
    int r = ((struct path *)q)->pathlen; 
    return (l - r);
}
 
path * yenksp(int graph[V][V], int src, int dest, int K)
{
  path * A = (path *)malloc((K+1)*sizeof(path));
  //B = []
  path B[MAX_CONTAINER_SIZE];
  //initialise
  for(int i=0;i<=K;i++)
    initialise(&(A[i]));
  for(int i=0;i<MAX_CONTAINER_SIZE;i++)
    initialise(&(B[i]));
  int topB=0;

  //A[0] = Dijkstra(Graph, source, sink);
  A[0]=dijkstra(graph,src,dest);



  //making graph backup for later restoration
  int old_graph[V][V];
    for(int y=0;y<V;y++)
      for(int z=0;z<V;z++)
        old_graph[y][z]=graph[y][z];

  //k-loop
  for(int k=1;k<=K;k++)
  {

    //finding no. of edges in A[k-1]
    int prevSize=0;
    while(A[k-1].pathlist[prevSize]!=-1)
      prevSize++;

    //i-loop
    for(int i=0;i<=prevSize-2;i++)
    {

      //spurNode = A[k-1].node(i);
      int spurNode=A[k-1].pathlist[i];
      //rootPath = A[k-1].nodes(0, i);
      path rootPath=subPath(graph, &(A[k-1]), 0,i);


      //for each path p in A:
      for(int x=0;x<=k-1;x++)
      {
        //checking if rootPath==A[x].nodes(0,i)
        int flag=1;
        for(int xx=0;xx<=i;xx++)
          if(rootPath.pathlist[xx]!=A[x].pathlist[xx])
            flag=0;

        //if yes
        if(flag==1)
          //removing egde p.edge(i,i+1)
          graph[A[x].pathlist[i]][A[x].pathlist[i+1]]=0;

      }

      //for each node rootPathNode in rootPath except spurNode: 
      for(int x=0;x<i;x++)
      {
        //remove rootPathNode from Graph;
        //(same as removing all edges from and to rootPathNode)
        for(int y=0;y<V;y++)
          {
            graph[y][rootPath.pathlist[x]]=0;
            graph[rootPath.pathlist[x]][y]=0;
          }
      }

      //spurPath = Dijkstra(Graph, spurNode, sink);
      path spurPath=dijkstra(graph,spurNode,dest);

      path totalPath;
      initialise(&totalPath);
      //totalPath = rootPath + spurPath;
      int yy=0,zz=0;
      while(rootPath.pathlist[zz]!=-1)
      {
        totalPath.pathlist[yy++]=rootPath.pathlist[zz++];
      }
      zz=1;//so that spurNode is not duplicated
      while(spurPath.pathlist[zz]!=-1)
      {
        totalPath.pathlist[yy++]=spurPath.pathlist[zz++];
      }
      totalPath.pathlen=rootPath.pathlen+spurPath.pathlen;


      //B.append(totalPath);
      B[topB++]=totalPath;

      //restore edges to Graph;
      //restore nodes in rootPath to Graph;
      for(int y=0;y<V;y++)
        for(int z=0;z<V;z++)
          graph[y][z]=old_graph[y][z];


    }//end i-loop

    //if B is empty:
        //break;
    if(topB==0)
      break;

    //B.sort();
    qsort((void*)B, topB, sizeof(path), comparator);
    //A[k] = B[0];
    A[k]=B[0];
    //B.pop();
    //(instead removing first element and replacing it with the largest element)
    B[0]=B[topB-1];
    initialise(&B[topB-1]);
    topB--;


  }//end k-loop

  return A;

}

// Driver Code
int main()
{
    //  Let us create the example
    // graph discussed above
    int graph[V][V] = {{0, 4, 0, 0, 0, 0, 0, 8, 0},
                       {4, 0, 8, 0, 0, 0, 0, 11, 0},
                        {0, 8, 0, 7, 0, 4, 0, 0, 2},
                        {0, 0, 7, 0, 9, 14, 0, 0, 0},
                        {0, 0, 0, 9, 0, 10, 0, 0, 0},
                        {0, 0, 4, 0, 10, 0, 2, 0, 0},
                        {0, 0, 0, 14, 0, 2, 0, 1, 6},
                        {8, 11, 0, 0, 0, 0, 1, 0, 7},
                        {0, 0, 2, 0, 0, 0, 6, 7, 0}
                    };
 
    //path P=dijkstra(graph, 0, 8);
    //printPath(P);

    int K=2;
    path * A = yenksp(graph, 0, 8, K);
    for(int i=0;i<=K;i++)
      printPath(A[i]);

    return 0;
}