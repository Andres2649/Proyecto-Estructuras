#ifndef PUNTO2D_H
#define PUNTO2D_H

// TAD Punto2D: representa una coordenada (fila, columna) en la matriz de bases
// del genoma para el componente 3 (grafos).

struct Punto2D {
    int fila;
    int col;

    Punto2D() : fila(0), col(0) {}
    Punto2D(int f, int c) : fila(f), col(c) {}

    // Que sea mismo i, mismo j
    bool operator==(const Punto2D& other) const {
        return fila == other.fila && col == other.col;
    }

    // Menor estricto
    bool operator<(const Punto2D& other) const {
        if (fila != other.fila) return fila < other.fila;
        return col < other.col;
    }
};

#endif // PUNTO2D_H
