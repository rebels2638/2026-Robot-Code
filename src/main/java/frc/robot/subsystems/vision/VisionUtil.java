package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.constants.FieldConstants;
import frc.robot.lib.BLine.FlippingUtil;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

final class VisionUtil {
    static final double FIELD_DIMENSION_MATCH_TOLERANCE_METERS = 0.005;
    private static final double CORRELATED_OBSERVATION_EPSILON_SECONDS = 0.005;

    static record CoalescedObservationsResult(
        List<PoseObservation> observations,
        int rawObservationCount,
        int coalescedObservationCount,
        int coalescedDropCount,
        int[] groupSizes,
        String[] winnerTypes,
        String[] decisionReasons
    ) {}

    private record ObservationSelection(PoseObservation winner, String reason) {}

    private VisionUtil() {}

    static double getFieldLengthMeters() {
        return FieldConstants.fieldLength;
    }

    static double getFieldWidthMeters() {
        return FieldConstants.fieldWidth;
    }

    static boolean isPoseWithinField(Pose3d pose) {
        return pose.getX() >= 0.0
            && pose.getX() <= getFieldLengthMeters()
            && pose.getY() >= 0.0
            && pose.getY() <= getFieldWidthMeters();
    }

    static boolean fieldDimensionsMatchFlippingUtil() {
        return Math.abs(getFieldLengthMeters() - FlippingUtil.fieldSizeX)
                <= FIELD_DIMENSION_MATCH_TOLERANCE_METERS
            && Math.abs(getFieldWidthMeters() - FlippingUtil.fieldSizeY)
                <= FIELD_DIMENSION_MATCH_TOLERANCE_METERS;
    }

    static String getFieldDimensionMismatchSummary() {
        return String.format(
            "Vision field dimensions mismatch B-Line FlippingUtil. FieldConstants=(%.6f, %.6f) "
                + "FlippingUtil=(%.6f, %.6f)",
            getFieldLengthMeters(),
            getFieldWidthMeters(),
            FlippingUtil.fieldSizeX,
            FlippingUtil.fieldSizeY
        );
    }

    static CoalescedObservationsResult coalesceCorrelatedObservations(List<PoseObservation> observations) {
        if (observations.isEmpty()) {
            return new CoalescedObservationsResult(List.of(), 0, 0, 0, new int[0], new String[0], new String[0]);
        }

        List<PoseObservation> sortedObservations = new ArrayList<>(observations);
        sortedObservations.sort(Comparator.comparingDouble(PoseObservation::timestamp));

        List<PoseObservation> coalescedObservations = new ArrayList<>();
        List<Integer> groupSizes = new ArrayList<>();
        List<String> winnerTypes = new ArrayList<>();
        List<String> decisionReasons = new ArrayList<>();
        List<PoseObservation> currentGroup = new ArrayList<>();
        PoseObservation previousObservation = null;

        for (PoseObservation observation : sortedObservations) {
            if (previousObservation == null
                || Math.abs(observation.timestamp() - previousObservation.timestamp())
                    <= CORRELATED_OBSERVATION_EPSILON_SECONDS) {
                currentGroup.add(observation);
                previousObservation = observation;
                continue;
            }

            appendCoalescedGroup(currentGroup, coalescedObservations, groupSizes, winnerTypes, decisionReasons);
            currentGroup.clear();
            currentGroup.add(observation);
            previousObservation = observation;
        }
        appendCoalescedGroup(currentGroup, coalescedObservations, groupSizes, winnerTypes, decisionReasons);

        return new CoalescedObservationsResult(
            coalescedObservations,
            observations.size(),
            coalescedObservations.size(),
            observations.size() - coalescedObservations.size(),
            toIntArray(groupSizes),
            winnerTypes.toArray(new String[0]),
            decisionReasons.toArray(new String[0])
        );
    }

    static PoseObservation choosePreferredObservation(
        PoseObservation firstObservation,
        PoseObservation secondObservation
    ) {
        return choosePreferredObservationWithReason(firstObservation, secondObservation).winner();
    }

    static String getSelectionReason(
        PoseObservation firstObservation,
        PoseObservation secondObservation
    ) {
        return choosePreferredObservationWithReason(firstObservation, secondObservation).reason();
    }

    private static ObservationSelection choosePreferredObservationWithReason(
        PoseObservation firstObservation,
        PoseObservation secondObservation
    ) {
        if (secondObservation.tagCount() != firstObservation.tagCount()) {
            return secondObservation.tagCount() > firstObservation.tagCount()
                ? new ObservationSelection(secondObservation, "higher_tag_count")
                : new ObservationSelection(firstObservation, "higher_tag_count");
        }

        int secondPriority = getObservationPriority(secondObservation.type());
        int firstPriority = getObservationPriority(firstObservation.type());
        if (secondPriority != firstPriority) {
            return secondPriority > firstPriority
                ? new ObservationSelection(secondObservation, "preferred_type")
                : new ObservationSelection(firstObservation, "preferred_type");
        }

        if (Double.compare(secondObservation.ambiguity(), firstObservation.ambiguity()) != 0) {
            return secondObservation.ambiguity() < firstObservation.ambiguity()
                ? new ObservationSelection(secondObservation, "lower_ambiguity")
                : new ObservationSelection(firstObservation, "lower_ambiguity");
        }

        if (Double.compare(
            secondObservation.averageTagDistance(),
            firstObservation.averageTagDistance()
        ) != 0) {
            return secondObservation.averageTagDistance() < firstObservation.averageTagDistance()
                ? new ObservationSelection(secondObservation, "lower_tag_distance")
                : new ObservationSelection(firstObservation, "lower_tag_distance");
        }

        if (Double.compare(secondObservation.timestamp(), firstObservation.timestamp()) != 0) {
            return secondObservation.timestamp() >= firstObservation.timestamp()
                ? new ObservationSelection(secondObservation, "latest_timestamp")
                : new ObservationSelection(firstObservation, "latest_timestamp");
        }

        return new ObservationSelection(secondObservation, "tie_break");
    }

    private static int getObservationPriority(PoseObservationType type) {
        return switch (type) {
            case MEGATAG_2 -> 2;
            case MEGATAG_1 -> 1;
            case PHOTONVISION -> 0;
        };
    }

    private static void appendCoalescedGroup(
        List<PoseObservation> group,
        List<PoseObservation> coalescedObservations,
        List<Integer> groupSizes,
        List<String> winnerTypes,
        List<String> decisionReasons
    ) {
        if (group.isEmpty()) {
            return;
        }

        PoseObservation winner = group.get(0);
        String reason = "single_observation";
        for (int i = 1; i < group.size(); i++) {
            ObservationSelection selection = choosePreferredObservationWithReason(winner, group.get(i));
            winner = selection.winner();
            reason = selection.reason();
        }

        coalescedObservations.add(winner);
        groupSizes.add(group.size());
        winnerTypes.add(winner.type().name());
        decisionReasons.add(reason);
    }

    private static int[] toIntArray(List<Integer> values) {
        int[] result = new int[values.size()];
        for (int i = 0; i < values.size(); i++) {
            result[i] = values.get(i);
        }
        return result;
    }
}
