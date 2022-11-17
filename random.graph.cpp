// g++-8 -O2 random.graph.cpp -o rg
// generate random graph according to specifications
// output to std::cout, file, and a dot file
#include <random>
#include <iostream>
#include <fstream>

int main()
{
    int N = 200;                // num cities
    double density = 0.4;       // 0.3 - 0.9
    int K = 10;                  // num recharges
    int M = N*(N-1)/2*density;  // num edges
    int max_len = N*N;

    std::random_device                      rd;
    std::mt19937                            gen( rd() );
    std::uniform_int_distribution<int>      node( 0,N-1 );      // other node
    std::uniform_int_distribution<int>      weight( 1, max_len ); // edge weight

    std::vector< std::vector< int > > d( N , std::vector< int >(N, 0) );

    // make graph connected  0--1--2-- ... --N-1

    int oldM = M;
    for ( int i=0; i<N-1; ++i ) {
        d[i][i+1] = weight( gen ); // edge backwards for connectedness
        --M;
    }

    while ( M>0 ) {
        int v1 = node( gen ); // attempt
        int v2 = node( gen ); // attempt
        if ( v1<v2 and v1 != v2 and d[v1][v2] == 0 ) {
            d[v1][v2] = weight( gen );
            --M;
        }
    }

    // to standard output
    std::cout << N << " " << K << " " << oldM << std::endl;
    for ( int i=0; i<N; ++i ) {
        for ( int j=0; j<N; ++j ) {
            if ( d[i][j] > 0 ) {
                std::cout << i << " " << j << " " <<  d[i][j] << std::endl;
            }
        }
    }

    // same to input file "in"
    std::ofstream infile( "in", std::ofstream::out );
    infile << N << " " << K << " " << oldM << "\n";
    for ( int i=0; i<N; ++i ) {
        for ( int j=0; j<N; ++j ) {
            if ( d[i][j] > 0 ) {
                infile << i << " " << j << " " <<  d[i][j] << std::endl;
            }
        }
    }

    // same to dot file
    // to generate PDF
    // dot -Tpdf in.dot -O
    // see "in.dot.pdf"
    std::ofstream dotfile( "in.dot", std::ofstream::out );
    dotfile << "graph g {\n";
    for ( int i=0; i<N; ++i ) {
        for ( int j=0; j<N; ++j ) {
            if ( d[i][j] > 0 ) {
                dotfile << i << "--" << j << " [label=\"" <<  d[i][j] << "\"];" << std::endl;
            }
        }
    }
    dotfile << "}\n";
}
