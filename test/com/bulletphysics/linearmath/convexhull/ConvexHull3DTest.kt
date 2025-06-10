package com.bulletphysics.linearmath.convexhull;

import com.bulletphysics.util.ObjectArrayList;
import org.junit.jupiter.api.Test;

import javax.vecmath.Vector3d;
import java.util.*;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

public class ConvexHull3DTest {

    // Helper to create Vector3f from floats
    private Vector3d v(float x, float y, float z) {
        return new Vector3d(x, y, z);
    }

    private static class ConvexHull3D {
        private final List<Vector3d> vertices;

        ConvexHull3D(List<Vector3d> pts) {
            HullDesc desc = new HullDesc();
            desc.vertices = new ObjectArrayList<>(pts.size());
            desc.vertices.addAll(pts);
            desc.maxVertices = pts.size();
            desc.vcount = pts.size();

            HullResult result = new HullResult();
            assertTrue(new HullLibrary().createConvexHull(desc, result));
            this.vertices = result.outputVertices;
        }

        public List<Vector3d> getVertices() {
            return vertices;
        }
    }

    @Test
    public void testTetrahedron() {
        List<Vector3d> pts = Arrays.asList(
                v(0, 0, 0), v(1, 0, 0), v(0, 1, 0), v(0, 0, 1)
        );
        ConvexHull3D hull = new ConvexHull3D(pts);

        List<Vector3d> vertices = hull.getVertices();

        // The hull of tetrahedron input should have exactly these vertices
        assertEquals(4, vertices.size());

        for (Vector3d p : pts) {
            boolean found = vertices.stream().anyMatch(v -> v.equals(p));
            assertTrue(found, "Hull should contain original point " + p);
        }
    }

    @Test
    public void testCubeCorners() {
        List<Vector3d> pts = Arrays.asList(
                v(0, 0, 0), v(1, 0, 0), v(1, 1, 0), v(0, 1, 0),
                v(0, 0, 1), v(1, 0, 1), v(1, 1, 1), v(0, 1, 1)
        );
        ConvexHull3D hull = new ConvexHull3D(pts);

        List<Vector3d> vertices = hull.getVertices();
        assertEquals(8, vertices.size());

        // All input points should be on hull
        for (Vector3d p : pts) {
            boolean found = vertices.stream().anyMatch(v -> v.equals(p));
            assertTrue(found);
        }
    }

    @Test
    public void testDuplicatePoints() {
        List<Vector3d> pts = Arrays.asList(
                v(0, 0, 0), v(1, 0, 0), v(0, 1, 0), v(0, 0, 1),
                v(0, 0, 0), v(1, 0, 0) // duplicates
        );
        ConvexHull3D hull = new ConvexHull3D(pts);

        List<Vector3d> vertices = hull.getVertices();

        // Should be 4 unique vertices
        assertEquals(4, vertices.size());

        // All unique points appear
        Set<Vector3d> uniqueInput = new HashSet<>(Arrays.asList(
                v(0, 0, 0), v(1, 0, 0), v(0, 1, 0), v(0, 0, 1)
        ));
        for (Vector3d p : uniqueInput) {
            boolean found = vertices.stream().anyMatch(v -> v.equals(p));
            assertTrue(found);
        }
    }

    @Test
    public void testRandomPointsConsistency() {
        Random rnd = new Random(123);
        List<Vector3d> pts = new ArrayList<>();
        for (int i = 0; i < 100; i++) {
            pts.add(new Vector3d(rnd.nextFloat(), rnd.nextFloat(), rnd.nextFloat()));
        }
        ConvexHull3D hull1 = new ConvexHull3D(pts);
        ConvexHull3D hull2 = new ConvexHull3D(pts);

        List<Vector3d> verts1 = hull1.getVertices();
        List<Vector3d> verts2 = hull2.getVertices();

        // Sort and compare vertices for equality
        Comparator<Vector3d> cmp = Comparator
                .<Vector3d>comparingDouble(v -> v.x)
                .thenComparingDouble(v -> v.y)
                .thenComparingDouble(v -> v.z);

        verts1.sort(cmp);
        verts2.sort(cmp);

        assertEquals(verts1.size(), verts2.size());

        for (int i = 0; i < verts1.size(); i++) {
            Vector3d a = verts1.get(i);
            Vector3d b = verts2.get(i);
            assertTrue(a.epsilonEquals(b, 1e-6f));
        }
    }
}