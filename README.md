//# Graph-data-structure
//graph every important algorithm code
import java.util.*;
import java.util.Queue;
import java.util.Stack;
public class Graph {
    static class Edge {
        int src;
        int dest;
        int wt;


        public Edge(int s, int d) {
            this.src = s;
            this.dest = d;
             
             
        }
    }

    public static void createGraph(ArrayList<Edge>[] graph) {
        for (int i = 0; i < graph.length; i++) {
            graph[i] = new ArrayList<>();
        }
        graph[0].add(new Edge(0,1));
        graph[0].add(new Edge(0,2));
        graph[0].add(new Edge(0,3));

        graph[1].add(new Edge(1,0));
        graph[1].add(new Edge(1,2));

        graph[2].add(new Edge(2,0));
        graph[2].add(new Edge(2,1));

        graph[3].add(new Edge(3,0));
        graph[3].add(new Edge(3,4));
       // graph[3].add(new Edge(3,5));

        graph[4].add(new Edge(4,3));
        //graph[4].add(new Edge(4,5));

        // graph[5].add(new Edge(5,3));
        // graph[5].add(new Edge(5,4));
        
    }

    public static void bfs(ArrayList<Edge>[] graph, int V,boolean vis[],int start) {
        Queue<Integer> q = new LinkedList<>();
        
        q.add(start);
        while (!q.isEmpty()) {
            int curr = q.remove();
            if (!vis[curr]) {
                System.out.print(curr + " ");
                vis[curr] = true;
                for (Edge e : graph[curr]) {
                    q.add(e.dest);
                }
            }
        }
    }

    public static void dfs(ArrayList<Edge> graph[],int curr,boolean vis[]){
        System.out.print(curr+" ");
        vis[curr]=true;
        for(int i=0;i<graph[curr].size();i++){
            Edge e=graph[curr].get(i);
            if(vis[e.dest]==false)
                dfs(graph, e.dest, vis);
        }
    }


    public static void printAllPath(ArrayList<Edge> graph[],boolean vis[],int curr,String path , int tar){
        if(curr==tar){
            System.out.println(path);
            return;
        }

        for(int i=0;i<graph[curr].size();i++){
            Edge e=graph[curr].get(i);
            if(!vis[e.dest]){
                vis[curr]=true;
                printAllPath(graph, vis, e.dest, path+e.dest, tar);
                vis[curr]=false;
            }
        }
    }
    public static boolean isCycledDirected(ArrayList<Edge> graph[],boolean vis[],int curr, boolean rec[]){
        vis[curr]= true;
        rec[curr]=true;
        for(int i=0;i<graph[curr].size();i++){
            Edge e=graph[curr].get(i);
            if(rec[e.dest]){
                return true;
            }else if(vis[e.dest]){
                if(isCycledDirected(graph, vis, e.dest, rec)){
                    return true;
                }
            }

        }
        rec[curr]=false;
        return false;
    }

    public static void topSortUtil(ArrayList<Edge> graph[],int curr,boolean vis[],Stack<Integer> stack){
        vis[curr]=true;
        for(int i=0;i<graph[curr].size();i++){
            Edge e=graph[curr].get(i);

            if(!vis[e.dest]){
                topSortUtil(graph, e.dest, vis, stack);
            }
        }

        stack.push(curr);
    }
    public static void topSort(ArrayList<Edge> graph[],int V){
        boolean vis[]=new boolean[V];
        Stack<Integer> stack=new Stack<>();

        for(int i=0;i<V;i++){
            if(!vis[i]){
                 topSortUtil(graph, i, vis, stack);
            }
        }
        while(!stack.isEmpty()){
            System.out.print(stack.pop()+" ");
        }
    }
    public static boolean isCycledUndirected(ArrayList<Edge> graph[],boolean vis[], int curr, int par){
        vis[curr]=true;

        for(int i=0;i<graph[curr].size();i++){
            Edge e=graph[curr].get(i);
            if(vis[e.dest] && e.dest!=par){
                return true;
            }
            else if(!vis[e.dest]){
                if(isCycledUndirected(graph, vis, e.dest, curr)){
                    return true;
                }
            }
        }

        return false;
    }

    public static class pair implements Comparable<pair>{
        int node;
        int dis;
        public pair(int n,int d){
            this.node=n;
            this.dis=d;
        }

        @Override
        public int compareTo(pair p2){
            return this.dis-p2.dis;
        }
    }

    public static void dijkstra(ArrayList<Edge> graph[],int src,int V){
        PriorityQueue<pair> pq=new PriorityQueue<>();
        int dist[]=new int[V];
        for(int i=0;i<V;i++){
            if(i!=src){
                dist[i]=Integer.MAX_VALUE;
            }
        }
        boolean vis[]=new boolean[V];

        pq.add(new pair(0, 0));
        while(!pq.isEmpty()){
            pair curr=pq.remove();
            if(!vis[curr.node]){
                vis[curr.node]=true;

                for(int i=0;i<graph[curr.node].size();i++){
                    Edge e=graph[curr.node].get(i);
                    int u=e.src;
                    int v=e.dest;
                    if(dist[u]+e.wt<dist[v]){
                        dist[v]=dist[u]+e.wt;
                        pq.add(new pair(v, dist[v]));
                    }
                }
            }
        }
        for(int i=0;i<V;i++){
            System.out.print(dist[i]+" ");
        }
        System.out.println();


    }
    public static void bellmanFord(ArrayList<Edge> graph[],int src,int V){
        int dist[]=new int[V];
        for(int i=0;i<V;i++){
            if(i!=src){
                dist[i]=Integer.MAX_VALUE;
            }
        }
        for(int k=0;k<V-1;k++){
            for(int i=0;i<V;i++){
                for(int j=0;j<graph[i].size();j++){
                    Edge e=graph[i].get(j);
                    int u=e.src;
                    int v=e.dest;

                    if(dist[u]!=Integer.MAX_VALUE && dist[u]+e.wt<dist[v]){
                        dist[v]=dist[u]+e.wt;
                    }
                }
            }
        }
        for(int i=0;i<dist.length;i++){
            System.out.println(dist[i]+" ");
        }
        System.out.println();
    }

     static class Pair implements Comparable<Pair>{
        int node;
        int cost;
        
        public Pair(int n,int c){
            this.node=n;
            this.cost=c;
        }

        @Override
        public int compareTo(Pair p2){
            return this.cost-p2.cost;
        }


    }

    public static void primsAlgo(ArrayList<Edge> graph[],int V){
        PriorityQueue<Pair> pq=new PriorityQueue<>();
        boolean vis[]=new boolean[V];
        pq.add(new Pair(0,0));
        
        int mstCost=0;

        while(!pq.isEmpty()){
            Pair curr=pq.remove();
            if(!vis[curr.node]){
                vis[curr.node]=true;
                mstCost+=curr.cost;

                for(int i=0;i<graph[curr.node].size();i++){
                    Edge e=graph[curr.node].get(i);
                    if(!vis[e.dest]){
                        pq.add(new Pair(e.dest,e.wt));
                    }
                }
            }
        }
        System.out.println("min cost of the mst ="+mstCost);
    }

    public static void  topsort(ArrayList<Edge> graph[],int curr,boolean vis[],Stack<Integer> s){
        vis[curr]=true;

        for(int i=0;i<graph[curr].size();i++){
            Edge e=graph[curr].get(i);
            if(!vis[e.dest]){
                topsort(graph, e.dest,vis,s);
            }
        }
        s.push(curr);
    }

    public static void Dfs(ArrayList<Edge> graph[],int curr,boolean vis[]){
        vis[curr]=true;
        System.out.print(curr+" ");
        for(int i=0;i<graph[curr].size();i++){
            Edge e=graph[curr].get(i);
            if(!vis[e.dest]){
                Dfs(graph, e.dest, vis);
            }
        }
    }

    public static void kosarajusAlgo(ArrayList<Edge> graph[], int V){
        //step 1
        Stack<Integer> s=new Stack<>();
        boolean vis[]=new boolean[V];
        for(int i=0;i<V;i++){
            if(!vis[i]){
                topsort(graph,i,vis,s);
            }
        }

        //step 2
        ArrayList<Edge> transposeGraph[]=new ArrayList[V];
        for(int i=0;i<graph.length;i++){
            vis[i]=false;

            transposeGraph[i]=new ArrayList<Edge>();

        }

        for(int i=0;i<V;i++){
            for(int j=0;j<graph[i].size();j++){
                Edge e=graph[i].get(j);
                transposeGraph[e.dest].add(new Edge(e.dest, e.src));
            }
        }

        //step 3
        while(!s.isEmpty()){
            int curr=s.pop();
            if(!vis[curr]){
                System.out.print("scc= ");
                Dfs(transposeGraph, curr, vis);
                System.out.println();
            }
        }

  }  

  public static void dfsTarjan(ArrayList<Edge> graph[],int curr,boolean vis[],int dt[],int low[],int time,int par){
    vis[curr]=true;
    dt[curr]=low[curr]=++time;

    for(int i=0;i<graph[curr].size();i++){
        Edge e=graph[curr].get(i);
        if(e.dest==par){
            continue;

        }
        else if(!vis[e.dest]){
            dfsTarjan(graph, e.dest, vis, dt, low, time, curr);
            low[curr]=Math.min(low[curr],low[e.dest]);
            if(dt[curr]<low[e.dest]){
                System.out.println("bridge is: "+curr+"----"+e.dest);
            } 
        }
        else{
            low[curr]=Math.min(low[curr],low[e.dest]);
        }
    }
  }

  public static void getBridge(ArrayList<Edge> graph[],int V){
    int dt[]=new int[V];
    int low[]=new int[V];
    int time=0;
    boolean vis[]=new boolean[V];

    for(int i=0;i<V;i++){
        if(!vis[i]){
            dfsTarjan(graph, i, vis,dt,low,time,-1);
        }
    }
  }


  public static void dfsAP(ArrayList<Edge>  graph[],int curr,int par,int dt[],int low[],boolean vis[],int time,boolean ap[]){
    vis[curr]=true;
    dt[curr]=low[curr]=++time;
    int children=0;

    for(int i=0;i<graph[curr].size();i++){
        Edge e=graph[curr].get(i);
        int neigh=e.dest;
        if(par==neigh){
            continue;
        }

        else if(vis[neigh]){
            low[curr]=Math.min(low[curr],dt[neigh]);

        }
        else{
            dfsAP(graph, neigh, curr, dt, low, vis, time, ap);
            low[curr]=Math.min(low[curr],low[neigh]);
            if(dt[curr]<=low[neigh] && par!=-1){
                ap[curr]=true;
            }
            children++;

        }
    }
    if(par==-1 && children>1){
        ap[curr]=true;
    }
  }

  public static void getAp(ArrayList<Edge> graph[], int V){
    int dt[]=new int[V];
    int low[]=new int[V];
    int time=0;
    boolean vis[]=new boolean[V];
    boolean ap[]=new boolean[V];

    for(int i=0;i<V;i++){
        if(!vis[i]){
            dfsAP(graph, i, -1, dt, low, vis, time, ap);
        }
    }
    for(int i=0;i<V;i++){
        if(ap[i]){
            System.out.println("ap :"+i);
        }
    }
  }
    public static void main(String args[]) {
        
    }
}
