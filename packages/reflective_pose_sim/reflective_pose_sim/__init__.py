"""Synthetic VLP-32C scans for detector development and node smoke-testing.

A package of its own rather than fixtures beside the tests, so the same scene
definitions drive both the offline test matrix and the live scene publisher.
It is a test and development dependency: nothing on the vehicle runtime path
may import it.
"""
