package main

import (
    "encoding/binary"
    "fmt"
    "github.com/jonas-p/go-shp"
    "github.com/xtgo/set"
    "io"
    "os"
    "path"
    "sort"
    "strconv"
)

type district struct {
    state int
    district int
    shape *shp.Polygon
}

type treenode struct {
    lower *treenode
    higher *treenode
    cutoff float64
    axis int
}

type tile struct {
    path []side
    bbox shp.Box
    // Districts with their shape restricted to the tile.
    clippedDistricts []district
}
type side int
const (
    low side = iota
    high
)

func main() {
    if len(os.Args) < 2 {
        fmt.Println("usage: district-tiler [shapefile.shp]")
        os.Exit(1)
    }
    filename := os.Args[1]
    namelen := len(filename)
    if namelen < 5 || filename[namelen-4:namelen] != ".shp" {
        fmt.Println("usage: district-tiler [shapefile.shp]")
        os.Exit(1)
    }

    fmt.Printf("Opening shapefile %s.\n", filename)
    districts := districtsFromShapefile(filename)
    totalPoints := 0
    for _, d := range districts {
        totalPoints += len(d.shape.Points)
    }
    fmt.Printf("Loaded %d districts comprising %d points.\n", len(districts), totalPoints)

    root := treeFromDistricts(districts, 4096)

    tiles := make([]*tile, 0, countLeaves(root))
    tiles = addTiles(tiles, root, shp.Box{-180, -90, 180, 90}, make([]side, 0))
    fmt.Printf("Created %d tiles.\n", len(tiles))

    for _, d := range districts {
        for _, t := range tiles {
            clipDistrict(d, t)
        }
    }

    fmt.Printf("Writing tiles..")
    err := os.Mkdir("tiles", 0755)
    if err != nil {
        fmt.Println("ERROR:", err)
        fmt.Println("...if you already have a tiles/ directory, please remove or rename it and run the command again.")
        os.Exit(1)
    }
    for _, t := range tiles {
        fmt.Printf(".")
        name := "tile-" + pathToString(t.path)
        file, err := os.Create(path.Join("tiles", name))
        if err != nil {
            fmt.Println("ERROR:", err)
            os.Exit(1)
        }
        writeTile(file, t)
        file.Close()
    }
    fmt.Printf("\n")

    fmt.Printf("Writing lookup.js...\n")
    inputjs, err := os.Open("lookup.js.template")
    if err != nil {
        fmt.Println("ERROR: The 'lookup.js.template' file is missing or can't be read.")
        os.Exit(1)
    }
    outputjs, err := os.Create(path.Join("tiles", "lookup.js"))
    if err != nil {
        fmt.Println("ERROR: Couldn't create a 'lookup.js' file in the tiles directory.")
        os.Exit(1)
    }
    io.WriteString(outputjs, "var tree = [")
    counter := 0
    queue := []*treenode{root}
    for len(queue) > 0 {
        node := queue[0]
        queue = queue[1:]
        if node == nil {
            continue
        }
        n := encodeCoordinate(node.axis, node.cutoff)
        counter += 1
        if node.axis == 1 {
            n |= 0x40000000
        }
        io.WriteString(outputjs, fmt.Sprintf("%d,", n))
        queue = append(queue, node.lower, node.higher)
    }
    io.WriteString(outputjs, "];\n")
    io.Copy(outputjs, inputjs)
    inputjs.Close()
    outputjs.Close()

    fmt.Println("Done!")
}

func countLeaves(node *treenode) int {
    if node == nil {
        return 1
    } else {
        return countLeaves(node.lower) + countLeaves(node.higher)
    }
}

func districtsFromShapefile(filename string) []district {
    r, err := shp.Open(filename)
    if err != nil {
        fmt.Println(err)
        os.Exit(1)
    }
    if r.GeometryType != shp.POLYGON {
        fmt.Println("The provided shapefile must contain polygons.")
        os.Exit(1)
    }

    stateField := -1
    districtField := -1
    for i, field := range r.Fields() {
        if string(field.Name[0:7]) == "STATEFP" {
            fmt.Println("Found STATEFP field.")
            stateField = i
        } else if string(field.Name[0:2]) == "CD" && string(field.Name[5:7]) == "FP" {
            fmt.Printf("Found %s field.\n", string(field.Name[0:7]))
            districtField = i
        }
    }
    if stateField == -1 {
        fmt.Println("I can't tell which state is which: no STATEFP field in shapefile")
        os.Exit(1)
    }
    if districtField == -1 {
        fmt.Println("I can't tell which district is which: no CD***FP field in shapefile")
        os.Exit(1)
    }

    var districts []district
    for r.Next() {
        i, s := r.Shape()
        cd := r.ReadAttribute(i, districtField)
        if cd == "ZZ" {
            // Ignore "ZZ" districts (which are inside bodies of water).
            continue
        }
        cdnum, err := strconv.Atoi(cd)
        if err != nil {
            fmt.Println("Couldn't parse district number: '%s'.\n", cd)
            os.Exit(1)
        }
        state := r.ReadAttribute(i, stateField)
        statenum, err := strconv.Atoi(state)
        if err != nil {
            fmt.Println("Couldn't parse state number: '%s'.\n", cd)
            os.Exit(1)
        }
        districts = append(districts, district{
            state: statenum,
            district: cdnum,
            shape: s.(*shp.Polygon),
        })
    }
    return districts
}

func treeFromDistricts(districts []district, limit int) *treenode {
    var xpts []shp.Point
    for _, d := range districts {
        xpts = append(xpts, d.shape.Points...)
    }
    sort.Sort(byX(xpts))
    xpts = xpts[:set.Uniq(byX(xpts))]
    ypts := make([]shp.Point, len(xpts))
    copy(ypts, xpts)
    sort.Sort(byY(ypts))
    return treeFromSortedPoints(xpts, ypts, limit)
}

func treeFromSortedPoints(xpts []shp.Point, ypts []shp.Point, limit int) *treenode {
    n := len(xpts) // == len(ypts)
    if n <= limit {
        return nil
    }
    node := &treenode{}
    xlo := xpts[:n/2]
    xhi := xpts[n/2:]
    ylo := ypts[:n/2]
    yhi := ypts[n/2:]
    if xpts[n-1].X - xpts[0].X > ypts[n-1].Y - ypts[0].Y {
        node.cutoff = xpts[n/2].X
        ylo = make([]shp.Point, 0, n/2)
        yhi = make([]shp.Point, 0, n/2)
        for _, p := range ypts {
            if p.X < node.cutoff || (p.X == node.cutoff && p.Y < xpts[n/2].Y) {
                ylo = append(ylo, p)
            } else {
                yhi = append(yhi, p)
            }
        }
        node.axis = 0
    } else {
        node.cutoff = ypts[n/2].Y
        xlo = make([]shp.Point, 0, n/2)
        xhi = make([]shp.Point, 0, n/2)
        for _, p := range xpts {
            if p.Y < node.cutoff || (p.Y == node.cutoff && p.X < ypts[n/2].X) {
                xlo = append(xlo, p)
            } else {
                xhi = append(xhi, p)
            }
        }
        node.axis = 1
    }
    node.lower = treeFromSortedPoints(xlo, ylo, limit)
    node.higher = treeFromSortedPoints(xhi, yhi, limit)
    return node
}

func addTiles(tiles []*tile, node *treenode, bbox shp.Box, path []side) []*tile {
    if node == nil {
        return append(tiles, &tile{
            path: path,
            bbox: bbox,
            clippedDistricts: make([]district, 0),
        })
    }

    pathL := make([]side, len(path) + 1)
    copy(pathL, path)
    pathL[len(path)] = low
    bboxL := bbox
    if node.axis == 0 {
        bboxL.MaxX = node.cutoff
    } else {
        bboxL.MaxY = node.cutoff
    }
    tiles = addTiles(tiles, node.lower, bboxL, pathL)

    pathH := append(path, high)
    bboxH := bbox
    if node.axis == 0 {
        bboxH.MinX = node.cutoff
    } else {
        bboxH.MinY = node.cutoff
    }
    tiles = addTiles(tiles, node.higher, bboxH, pathH)

    return tiles
}

func clipDistrict(d district, t *tile) {
    if !intersects(d.shape.Box, t.bbox) {
        return
    }
    clipped := district{
        state: d.state,
        district: d.district,
        shape: &shp.Polygon{},
    }
    for i, part := range d.shape.Parts {
        next := d.shape.NumPoints
        if i + 1 < len(d.shape.Parts) {
            next = d.shape.Parts[i + 1]
        }
        clipPart(clipped.shape, d.shape.Points[part:next], t.bbox)
    }
    clipped.shape.NumParts = int32(len(clipped.shape.Parts))
    clipped.shape.NumPoints = int32(len(clipped.shape.Points))
    if clipped.shape.NumParts > 0 {
        t.clippedDistricts = append(t.clippedDistricts, clipped)
    }
}

func clipPart(dest *shp.Polygon, src []shp.Point, box shp.Box) {
    // Remove the duplicate point.
    result := src[:len(src)-1]
    result = clipEdge(result, 0, high, box.MinX)
    result = clipEdge(result, 0, low, box.MaxX)
    result = clipEdge(result, 1, high, box.MinY)
    result = clipEdge(result, 1, low, box.MaxY)
    if len(result) == 0 {
        return
    }
    // Add the duplicate back in.
    result = append(result, result[0])
    dest.Parts = append(dest.Parts, int32(len(dest.Points)))
    dest.Points = append(dest.Points, result...)
}

// This is the Sutherland-Hodgeman clipping algorithm.
func clipEdge(src []shp.Point, axis int, side side, value float64) []shp.Point {
    if len(src) == 0 {
        return nil
    }
    result := make([]shp.Point, 0, len(src))
    a := src[len(src)-1]
    for _, b := range src {
        if whichSide(axis, value, b) == side {
            if whichSide(axis, value, a) != side {
                result = append(result, intersectEdge(axis, value, a, b))
            }
            result = append(result, b)
        } else if whichSide(axis, value, a) == side {
            result = append(result, intersectEdge(axis, value, a, b))
        }
        a = b
    }
    return result
}

func whichSide(axis int, value float64, p shp.Point) side {
    v := p.X
    if axis == 1 {
        v = p.Y
    }
    if v < value {
        return low
    } else {
        return high
    }
}

func intersectEdge(axis int, value float64, a shp.Point, b shp.Point) shp.Point {
    if axis == 0 {
        return shp.Point{
            value,
            a.Y + (b.Y - a.Y) * (value - a.X) / (b.X - a.X),
        }
    } else {
        return shp.Point{
            a.X + (b.X - a.X) * (value - a.Y) / (b.Y - a.Y),
            value,
        }
    }
}

func intersects(a shp.Box, b shp.Box) bool {
    return (((a.MinX <= b.MinX && b.MinX <= a.MaxX) ||
             (b.MinX <= a.MinX && a.MinX <= b.MaxX)) &&
            ((a.MinY <= b.MinY && b.MinY <= a.MaxY) ||
             (b.MinY <= a.MinY && a.MinY <= b.MaxY)))
}

func pathToString(path []side) string {
    num := 0
    for _, s := range path {
        num *= 2
        num += int(s)
    }
    return fmt.Sprintf("%04d", num)
}

// Points are encoded as 30-bit unsigned integers.
func encodeCoordinate(axis int, c float64) uint32 {
    d := float64(360)
    if axis == 1 { d = 180 }
    return uint32((c / d + 0.5) * ((1<<30) - 1))
}
func encodePoint(p shp.Point) (uint32, uint32) {
    return encodeCoordinate(0, p.X), encodeCoordinate(1, p.Y)
}

// 10xxxxxx xxxxxxxx xxxxxxxx xxxxxxxx - 30-bit coordinate
// 0xxxxxxx xxxxxxxx                   - 15-bit delta from last coordinate
// 11111111 00000000 ssssssss dddddddd - new district with 8-bit state and district numbers
// 11111111 11111111                   - new part in same district
func writeTile(w io.Writer, tile *tile) {
    for _, d := range tile.clippedDistricts {
        if d.state >= 256 || d.state < 0 {
            fmt.Printf("ERROR: state number '%d' out of range (a maximum of 256 states are supported).\n", d.state)
            os.Exit(1)
        }
        if d.district >= 256 || d.district < 0 {
            fmt.Printf("ERROR: district number '%d' out of range (a maximum of 256 districts per state are supported).\n", d.state)
            os.Exit(1)
        }
        binary.Write(w, binary.BigEndian, uint16(0xff00))
        binary.Write(w, binary.BigEndian, uint8(d.state))
        binary.Write(w, binary.BigEndian, uint8(d.district))
        for i, part := range d.shape.Parts {
            if i != 0 {
                binary.Write(w, binary.BigEndian, uint16(0xffff))
            }
            next := d.shape.NumPoints
            if i + 1 < len(d.shape.Parts) {
                next = d.shape.Parts[i + 1]
            }
            writePart(w, d.shape.Points[part:next])
        }
    }
}

func writePart(w io.Writer, points []shp.Point) {
    var lastX, lastY uint32
    for _, p := range points {
        x, y := encodePoint(p)
        if !writeDelta(w, lastX, x) {
            writeAbsolute(w, x)
        }
        if !writeDelta(w, lastY, y) {
            writeAbsolute(w, y)
        }
        lastX = x
        lastY = y
    }
}

func writeDelta(w io.Writer, prev uint32, val uint32) bool {
    delta := int32(val) - int32(prev)
    if delta < (1<<14) && delta >= -(1<<14) {
        d := uint16(delta)
        if delta < 0 {
            d = 1 + ^uint16(-delta)
        }
        d &= 0x7fff
        binary.Write(w, binary.BigEndian, d)
        return true
    }
    return false
}

func writeAbsolute(w io.Writer, a uint32) {
    a &= 0x3fffffff
    a |= 0x80000000
    binary.Write(w, binary.BigEndian, a)
}

type byX []shp.Point
func (s byX) Len() int { return len(s) }
func (s byX) Swap(i, j int) { s[i], s[j] = s[j], s[i] }
func (s byX) Less(i, j int) bool {
    a, b := s[i], s[j]
    return a.X < b.X || (a.X == b.X && a.Y < b.Y)
}
type byY []shp.Point
func (s byY) Len() int { return len(s) }
func (s byY) Swap(i, j int) { s[i], s[j] = s[j], s[i] }
func (s byY) Less(i, j int) bool {
    a, b := s[i], s[j]
    return a.Y < b.Y || (a.Y == b.Y && a.X < b.X)
}
